// =====================================================================
//  Refuel Servicer 2 — 블랙필 A (도킹·재급유 모듈, BULLDOZER 버전)
//  WeAct Black Pill V3.x (STM32F411CEU6)
//  제어: 릴레이 8(밸브) + 불도저 스텝모터(NK266/SBD-20, 볼스크류 고토크 이송)
//
//  2026-06-26 변경:
//   - 그리퍼(집게) → 불도저(볼스크류 이송)로 교체.
//   - 결선 PNP (COM=GND): push-pull, STEP HIGH 펄스, EN LOW=활성(SBD-20 실측).
//   - 모터 파라미터를 런타임 레지스트리화 → GCS 라이브 튜닝(SETP/GETP/PARAMS).
//     cFS param_lib가 SETP로 값 주입(재빌드 없이). 영속성=cFS측 SAVE/LOAD.
//
//  통신: USB CDC 115200, ASCII 라인 명령 (HELP 참조)
//  플래시: USB serial "BOOT\n" → ROM DFU 자체 점프 → dfu-util 재플래시
//
//  ⚠️ 안전 규칙 (IREC gimbal_fw에서 검증된 원칙):
//   1. reboot_to_bootloader() 는 SACRED — 동작 검증된 코드, 손대지 말 것.
//   2. loop()의 serial 폴링 앞에 blocking 코드 넣지 말 것.
//   3. USB CDC 초기화/clock 설정 손대지 말 것.
//   4. 큰 변경 후: LED 1Hz → echo → BOOT 동작 셋 다 확인 후 다음 작업.
// =====================================================================
#include <Arduino.h>
#include <math.h>      // powf — 익스포넨셜 가감속 프로파일

// =====================================================================
//  핀맵 (space_challenge_blackpill.md 확정본)
//  회피핀: PA11/12(USB) PA13/14(SWD) PA0(KEY) PA4-7(SPI flash) PC13(LED)
// =====================================================================
constexpr int LED_PIN = PC13;                       // 온보드 LED (active-low)

// 릴레이 8 (솔레노이드 밸브) — 릴레이 모듈, push-pull GPIO
constexpr int RELAY_PINS[8] = { PB12, PB13, PB14, PB15, PA8, PA9, PA10, PB5 };

// 불도저 스텝 (SBD-20 드라이버, PNP/active-high → push-pull, COM=GND)
constexpr int PIN_STEP = PB6;   // STP (펄스)
constexpr int PIN_DIR  = PB7;
constexpr int PIN_EN   = PB8;

// 홀센서 (A3144E 모듈 DO) — 불도저 축 자석 2개 → 1회전당 2펄스. 위치 피드백/탈조 검증용.
//  PB10: 5V-tolerant(FT). DO=오픈컬렉터+모듈 풀업 → INPUT. FALLING=자석 S극 1회 통과.
constexpr int PIN_HALL = PB10;

// Hall pulse counter lives here (not in the hall section below) because the
// motor's stall guard reads it while stepping.  ISR + commands stay together
// further down.
volatile uint32_t hall_count   = 0;     // cumulative pulses (magnet passes)
volatile uint32_t hall_last_us = 0;     // noise guard timestamp
volatile uint32_t hall_rejected = 0;    // edges dropped as impossible (noise)

constexpr uint32_t SYSTEM_MEMORY_BASE = 0x1FFF0000UL;

// =====================================================================
//  BOOT / ROM DFU 재진입 — SACRED. IREC gimbal_fw v0 검증 시퀀스 그대로.
// =====================================================================
static void reboot_to_bootloader() {
  __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
  USB_OTG_FS->GCCFG &= ~USB_OTG_GCCFG_PWRDWN;
  for (volatile uint32_t i = 0; i < 10000000UL; ++i) { __NOP(); }

  __disable_irq();
  HAL_RCC_DeInit();
  HAL_DeInit();

  SysTick->CTRL = 0;
  SysTick->LOAD = 0;
  SysTick->VAL  = 0;
  for (int i = 0; i < 8; ++i) {
    NVIC->ICER[i] = 0xFFFFFFFFU;
    NVIC->ICPR[i] = 0xFFFFFFFFU;
  }
  SCB->VTOR = 0;

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __DSB();
  __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();
  __DSB();

  __enable_irq();

  uint32_t bl_sp = *(volatile uint32_t *)(SYSTEM_MEMORY_BASE + 0);
  uint32_t bl_pc = *(volatile uint32_t *)(SYSTEM_MEMORY_BASE + 4);
  __set_MSP(bl_sp);
  ((void (*)(void))bl_pc)();
  while (1) {}
}

// =====================================================================
//  릴레이 (밸브) — GPIO ON/OFF (유지)
//  ⚠️ RELAY_ACTIVE_HIGH: 릴레이 모듈 트리거 극성 (실물 확인 후 확정)
//     메모리상 AOCS 보드는 active-low 옵토 릴레이였음 → 실물 검증 필요.
// =====================================================================
constexpr bool RELAY_ACTIVE_HIGH = false;    // active-low 옵토 릴레이 모듈 (실측 2026-06-26: HIGH=OFF→부팅 시 전부 OFF, LOW=ON)
static uint8_t relay_state = 0;              // bit i = 릴레이 i+1 (1=open)

static void relay_write(int idx, bool on) {  // idx 0..7
  bool level = RELAY_ACTIVE_HIGH ? on : !on;
  digitalWrite(RELAY_PINS[idx], level ? HIGH : LOW);
  if (on) relay_state |= (1 << idx); else relay_state &= ~(1 << idx);
}
static void relay_all(uint8_t mask) {
  for (int i = 0; i < 8; i++) relay_write(i, (mask >> i) & 1);
}

// =====================================================================
//  모터 파라미터 — 런타임 레지스트리 (GCS 라이브 튜닝, SETP/GETP/PARAMS)
//  cFS param_lib ↔ 블랙필 SETP 동기. 파라미터 추가 = PARAM_REG[]에 한 줄.
//  값은 RAM(재부팅 시 default) — 영속성은 cFS param_lib SAVE/LOAD가 담당.
// =====================================================================
struct MotorParams {
  float    steps_per_mm  = 400.0f;   // mm당 스텝 = 800분주 ÷ 리드 2mm (refuel_servicer_3 후배 튜닝값 반영, 2026-06-26)
  uint32_t start_delay   = 1100;     // 출발 half-delay [us] (정지마찰 돌파)
  uint32_t target_delay  = 150;      // 최고속 half-delay [us]
  uint32_t accel_steps   = 1000;     // 가속/감속 구간 스텝 수
  float    stroke_mm     = 65.0f;    // GRIP OPEN/CLOSE 행정 [mm] (실측 2026-08-10)
  uint8_t  dir_invert    = 0;        // 0=정상(+ =HIGH), 1=방향 반전
  // 0=never hold, 1=always hold, 2=hold ONLY after a closing move (gripping).
  // 2 is the operating mode: the clamp must not be back-driven while docked,
  // but open/calibration moves should not cook the motor holding air.
  uint8_t  hold          = 2;
  uint8_t  accel_profile = 0;        // 0=linear, 1=exponential
  float    exp_curve     = 2.0f;     // exp 곡률(>1: 초반 완만→후반 급가속)
  // Stall guard (2026-08-10): compare commanded travel against the hall-measured
  // travel and stop the move when they diverge -- catches a jam, a slipping
  // coupling, or step loss from a sagging motor battery.  Ships DISABLED so the
  // rig behaves exactly as before until the shaft magnets are mounted and the
  // pulses-per-mm scale is verified on the bench; a miswired hall would
  // otherwise read "no motion" and abort every move.
  uint8_t  stall_en      = 0;        // 0=off (default), 1=guard active
  float    stall_mm      = 3.0f;     // stop when commanded-vs-measured lag exceeds this [mm]
  float    hall_per_mm   = 2.0f;     // hall pulses per mm (BENCH-VERIFIED 2.0: 60 mm = 120 pulses, 2026-08-10)
  // Minimum spacing between accepted hall edges [us].  Real magnet passes are
  // far apart -- at 8.3 mm/s and 2 pulses/mm they arrive every ~60 ms -- so a
  // guard of a few ms rejects driver-noise bursts and contact chatter without
  // ever dropping a genuine pulse.  Raise the motor speed and this must come
  // down, which is why it is tunable rather than baked in.  (2026-08-10: logs
  // showed 20 counts in one second, above what the mechanism can physically
  // produce, with the old 0.5 ms guard.)
  uint32_t hall_guard_us = 20000;
  // Speed-derived hall filter.  A fixed guard cannot work: the interval between
  // genuine pulses changes from ~440 ms at the start of the ramp to ~60 ms at
  // top speed, so one number is either useless early or too tight later.  The
  // step generator already knows the current period, so derive the floor from it
  // and reject anything that arrives sooner than the shaft can possibly deliver.
  // Margin covers magnet spacing that is not exactly 180 deg; the magnets here
  // are close to it, so 0.75 rather than a loose 0.6.
  uint8_t  hall_dyn_en   = 1;        // 0 = fixed guard only (old behaviour)
  // 0.5, verified on the rig 2026-08-11: three consecutive 60 mm round trips
  // passed 119-120/120 pulses in BOTH directions (error <= 1 pulse) while
  // rejecting 60-249 noise edges per move.  0.75 discarded genuine pulses under
  // heavy coupled noise (open direction read 46/60 mm) -- do not tighten.
  float    hall_dyn_marg = 0.5f;     // fraction of the ideal interval to demand
};
static MotorParams mp;

enum PType { PT_F32, PT_U32, PT_U8 };
struct ParamReg { const char* name; PType type; void* ptr; double mn, mx; };

// ⬇️ 파라미터 추가는 여기 한 줄. 이름은 cFS param_lib 등록명과 1:1로 맞출 것.
static const ParamReg PARAM_REG[] = {
  { "steps_per_mm",  PT_F32, &mp.steps_per_mm,  1.0,   100000.0 },
  { "start_delay",   PT_U32, &mp.start_delay,   50.0,  60000.0  },
  { "target_delay",  PT_U32, &mp.target_delay,  20.0,  60000.0  },
  { "accel_steps",   PT_U32, &mp.accel_steps,   0.0,   1000000.0},
  { "stroke_mm",     PT_F32, &mp.stroke_mm,     0.0,   100000.0 },
  { "dir_invert",    PT_U8,  &mp.dir_invert,    0.0,   1.0      },
  { "hold",          PT_U8,  &mp.hold,          0.0,   2.0      },
  { "accel_profile", PT_U8,  &mp.accel_profile, 0.0,   1.0      },
  { "exp_curve",     PT_F32, &mp.exp_curve,     0.1,   10.0     },
  { "stall_en",      PT_U8,  &mp.stall_en,      0.0,   1.0      },
  { "stall_mm",      PT_F32, &mp.stall_mm,      0.5,   1000.0   },
  { "hall_per_mm",   PT_F32, &mp.hall_per_mm,   0.01,  1000.0   },
  { "hall_guard_us", PT_U32, &mp.hall_guard_us, 0.0,   1000000.0},
  { "hall_dyn_en",   PT_U8,  &mp.hall_dyn_en,   0.0,   1.0      },
  { "hall_dyn_marg", PT_F32, &mp.hall_dyn_marg, 0.05,  0.95     },
};
static const int PARAM_N = sizeof(PARAM_REG) / sizeof(PARAM_REG[0]);

static const ParamReg* param_find(const char* name) {
  for (int i = 0; i < PARAM_N; i++)
    if (!strcasecmp(name, PARAM_REG[i].name)) return &PARAM_REG[i];
  return nullptr;
}
static double param_read(const ParamReg* p) {
  switch (p->type) {
    case PT_F32: return *(float*)p->ptr;
    case PT_U32: return *(uint32_t*)p->ptr;
    default:     return *(uint8_t*)p->ptr;
  }
}
static void param_write(const ParamReg* p, double v) {
  if (v < p->mn) v = p->mn;  else if (v > p->mx) v = p->mx;
  switch (p->type) {
    case PT_F32: *(float*)p->ptr    = (float)v;                break;
    case PT_U32: *(uint32_t*)p->ptr = (uint32_t)(v + 0.5);     break;
    default:     *(uint8_t*)p->ptr  = (uint8_t)(v + 0.5);      break;
  }
}
static void param_print(const ParamReg* p) {   // "name=value"
  Serial.print(p->name); Serial.print('=');
  if (p->type == PT_F32) Serial.print(*(float*)p->ptr, 3);
  else if (p->type == PT_U32) Serial.print(*(uint32_t*)p->ptr);
  else Serial.print(*(uint8_t*)p->ptr);
}

// =====================================================================
//  불도저 스텝모터 (NK266 / SBD-20) — 논블로킹 위치제어 + 사다리꼴/지수 가감속
//  PNP / active-high (COM=GND): STEP HIGH=펄스, EN LOW=활성. DIR HIGH=정(+), dir_invert로 반전.
//  제어 파라미터는 전부 mp.* (런타임). 펄스 토글 로직은 검증된 그대로.
// =====================================================================
enum MotorState { MOT_IDLE, MOT_MOVING };
static MotorState   mot_state     = MOT_IDLE;
static long         mot_total     = 0;        // 총 스텝
static long         mot_i         = 0;        // 현재 스텝
static long         mot_accel     = 1000;     // 이번 이동의 가감속 구간 (mp.accel_steps 기반)
static uint32_t     mot_cur_delay = 1100;     // 현재 half-delay (us)
static uint32_t     mot_last_us   = 0;
static bool         mot_pin_high  = false;    // STEP 현재 레벨 (active-high)
static float        mot_dist_mm   = 0.0f;
static uint32_t     mot_t0        = 0;
static uint32_t     mot_hall0     = 0;        // hall_count when this move started
static uint8_t      mot_stalled   = 0;        // last move ended on the stall guard
static const char*  mot_stop_why  = "";       // why the last move ended (STAT)
// Worst gap between consecutive step pulses in the last move [us].  Pulses are
// produced from loop(), so anything that blocks loop() -- a long Serial reply,
// for instance -- stretches this gap, and a stepper running near speed can slip
// when the pulse train pauses and restarts.  Reporting the measured worst case
// turns "the motor is inconsistent" into a number that either shows the stall or
// rules it out.
static uint32_t     mot_gap_max   = 0;
static uint32_t     mot_prev_step = 0;

// Travel this move as the hall sensor actually saw it [mm].  Pulses are
// direction-blind (single sensor, no quadrature), so this is a magnitude.
static float mot_hall_mm() {
  if (mp.hall_per_mm <= 0.0f) return 0.0f;
  uint32_t now;
  noInterrupts(); now = hall_count; interrupts();
  // Guard the unsigned subtraction: HALLRST zeroes the counter while a baseline
  // from an earlier move is still held, and 0 - baseline wraps to ~4.29e9 (seen
  // on the ground as hall_mm = 4294966272).  Below the baseline means "nothing
  // measured since the reset".
  if (now < mot_hall0) return 0.0f;
  return (float)(now - mot_hall0) / mp.hall_per_mm;
}

// Commanded travel so far [mm] (absolute).
static float mot_cmd_mm() {
  if (mp.steps_per_mm <= 0.0f) return 0.0f;
  return (float)mot_i / mp.steps_per_mm;
}

static void motor_enable(bool on) {
  // SBD-20 실측(2026-06-26): EN 신호 인가(옵토 ON)=비활성 → 활성=옵토 OFF=EN 핀 LOW.
  digitalWrite(PIN_EN, on ? LOW : HIGH);
}

// 가속 프로파일에 따른 현재 스텝 half-delay [us].
//  t = 가감속 진행도(0=출발, 1=최고속). linear=선형보간, exp=지수보간(속도 지수 증가).
static uint32_t mot_delay_at(long i) {
  if (mot_accel <= 0) return mp.target_delay;
  float t;
  if (i < mot_accel)                    t = (float)i / (float)mot_accel;             // 가속
  else if (i > mot_total - mot_accel)   t = (float)(mot_total - i) / (float)mot_accel; // 감속
  else                                  return mp.target_delay;                       // 정속
  if (t < 0.0f) t = 0.0f; else if (t > 1.0f) t = 1.0f;

  const float s = (float)mp.start_delay, g = (float)mp.target_delay;
  if (mp.accel_profile == 1 && s > 0.0f && g > 0.0f) {     // exponential
    float tt = powf(t, mp.exp_curve);                       // 곡률(>1: 초반 완만)
    return (uint32_t)(s * powf(g / s, tt));                 // t=0→s, t=1→g
  }
  return (uint32_t)(s + (g - s) * t);                       // linear
}

static void motor_move(float dist_mm) {
  if (mot_state == MOT_MOVING) { Serial.println(F("ERR MOVE busy (STOP first)")); return; }
  long total = (long)((dist_mm >= 0 ? dist_mm : -dist_mm) * mp.steps_per_mm);
  if (total <= 0) { Serial.println(F("ERR MOVE bad dist")); return; }

  motor_enable(true);
  bool dir_high = (dist_mm >= 0);            // 기본: +방향 = HIGH
  if (mp.dir_invert) dir_high = !dir_high;   // 방향 반전 파라미터
  digitalWrite(PIN_DIR, dir_high ? HIGH : LOW);
  delayMicroseconds(5);                      // DIR setup

  long as       = (long)mp.accel_steps;
  mot_total     = total;
  mot_accel     = (total < as * 2) ? (total / 2) : as;
  mot_i         = 0;
  mot_cur_delay = mot_delay_at(0);
  mot_pin_high  = false;
  digitalWrite(PIN_STEP, LOW);               // idle LOW (active-high)
  mot_last_us   = micros();
  mot_dist_mm   = dist_mm;
  mot_t0        = millis();
  noInterrupts(); mot_hall0 = hall_count; interrupts();   // stall-guard baseline
  mot_stalled   = 0;
  mot_stop_why  = "";
  mot_gap_max   = 0;
  mot_prev_step = 0;
  mot_state     = MOT_MOVING;

  Serial.print(F(">> MOVE ")); Serial.print(dist_mm); Serial.print(F("mm = "));
  Serial.print(dist_mm >= 0 ? F("+") : F("-"));
  Serial.print(total); Serial.print(F(" steps, accel=")); Serial.print(mot_accel);
  Serial.print(F(", profile=")); Serial.println(mp.accel_profile ? F("exp") : F("lin"));
}

// Keep torque on after this move?  Mode 2 holds only when the move CLOSED
// (negative travel = gripping) -- so a contact stop mid-close stays clamped,
// while opens and calibration moves free the coils and stay cool.
static bool hold_now() {
  return mp.hold == 1 || (mp.hold == 2 && mot_dist_mm < 0.0f);
}

static void motor_stop(const char* why) {
  mot_state = MOT_IDLE;
  digitalWrite(PIN_STEP, LOW);               // idle LOW
  if (!hold_now()) motor_enable(false);
  mot_stop_why = why;
  Serial.print(F(">> MOVE stop (")); Serial.print(why); Serial.println(F(")"));
}

// loop()에서 매번 호출 — 논블로킹 STEP 펄스 + 가감속 프로파일
//  half-delay마다 STEP 토글: LOW→HIGH(rising=스텝) → HIGH→LOW(펄스 완료, 카운트).
static void motor_update() {
  if (mot_state != MOT_MOVING) return;
  uint32_t now = micros();
  if ((uint32_t)(now - mot_last_us) < mot_cur_delay) return;
  mot_last_us = now;
  mot_pin_high = !mot_pin_high;
  digitalWrite(PIN_STEP, mot_pin_high ? HIGH : LOW);   // HIGH=펄스 active
  if (!mot_pin_high) {                                 // 펄스 1개 완료 (HIGH→LOW)
    if (mot_prev_step != 0) {                          // worst pulse-to-pulse gap
      uint32_t gap = now - mot_prev_step;
      if (gap > mot_gap_max) mot_gap_max = gap;
    }
    mot_prev_step = now;
    mot_i++;
    if (mot_i >= mot_total) {
      mot_state = MOT_IDLE;
      digitalWrite(PIN_STEP, LOW);
      if (!hold_now()) motor_enable(false);
      mot_stop_why = "done";
      float secs = (millis() - mot_t0) / 1000.0f;
      Serial.print(F(">> MOVE ")); Serial.print(mot_dist_mm);
      Serial.print(F("mm done (")); Serial.print(secs, 2); Serial.println(F("s)"));
    } else {
      mot_cur_delay = mot_delay_at(mot_i);
      // Stall guard: commanded travel has run ahead of what the hall sensor saw
      // by more than stall_mm, so the shaft is not following.  Stop rather than
      // keep pushing -- a jammed 11 kgf ball screw builds force fast.  The lag
      // normally sits under one pulse (quantisation), so stall_mm carries real
      // margin above that.
      if (mp.stall_en && (mot_cmd_mm() - mot_hall_mm()) > mp.stall_mm) {
        mot_stalled = 1;
        motor_stop("stall");
      }
    }
  }
}

// =====================================================================
//  홀센서 (A3144E) — 자석 통과(회전) 카운트. 위치 피드백 / 탈조 검증용.
//  DO FALLING 엣지 = 자석 S극 1회 통과. 자석 2개 → 1회전당 2펄스.
//  ISR은 카운트만 (Serial 금지). 라이브 출력은 loop에서 hall_mon일 때만.
// =====================================================================
static bool       hall_mon     = false; // HALLMON 1 → loop에서 엣지 라이브 출력

// Smallest interval a genuine pulse can have RIGHT NOW [us].
//
// While stepping, the shaft turns at a rate the step generator dictates, so the
// spacing of magnet passes follows from it:
//     steps per pulse = steps_per_mm / hall_per_mm
//     ideal interval  = steps per pulse * step period (2 * current half-delay)
// That is ~440 ms at the start of the ramp and ~60 ms at top speed, which is
// why a single fixed number cannot serve both.  Anything arriving sooner than a
// margin of that did not come from a magnet.
//
// Idle falls back to the fixed guard: with no commanded motion there is no rate
// to derive, and hand-turning the shaft during calibration must still count.
static uint32_t hall_min_interval() {
  if (!mp.hall_dyn_en || mot_state != MOT_MOVING) return mp.hall_guard_us;
  if (mp.hall_per_mm <= 0.0f || mp.steps_per_mm <= 0.0f) return mp.hall_guard_us;
  float steps_per_pulse = mp.steps_per_mm / mp.hall_per_mm;
  float ideal = steps_per_pulse * 2.0f * (float)mot_cur_delay;
  float floor_us = ideal * mp.hall_dyn_marg;
  if (floor_us < 0.0f) floor_us = 0.0f;
  return (uint32_t)floor_us;
}

// NOTE: a pulse-width check was tried here (reject edges whose LOW time is too
// short) and REVERTED 2026-08-11 after it made travel readings worse on the
// rig in both its loop-polled and edge-timed forms (59.5 mm true -> 35.5/46.5).
// The interval filter alone, margin 0.6, matched both directions at 59.5/60.
// Do not reintroduce a width check without a bench scope trace first.
static void hall_isr() {
  uint32_t now = micros();
  if ((uint32_t)(now - hall_last_us) < hall_min_interval()) {
    hall_rejected++;          // impossible spacing -> noise, not a magnet
    return;
  }
  hall_last_us = now;
  hall_count++;
}

// =====================================================================
//  명령 인터페이스 (ASCII, USB CDC)
// =====================================================================
static void print_help() {
  Serial.println(F("=== Refuel Servicer 2 (Bulldozer) — commands ==="));
  Serial.println(F("  HELP                 이 목록"));
  Serial.println(F("  STAT                 릴레이/불도저 상태"));
  Serial.println(F("  BOOT                 ROM DFU 재진입 (재플래시)"));
  Serial.println(F("  -- 릴레이(밸브) 8 --"));
  Serial.println(F("  RLY <1-8> <0|1>      릴레이 open(1)/close(0)"));
  Serial.println(F("  RLYALL <0-255>       8개 비트마스크 일괄"));
  Serial.println(F("  -- 불도저 스텝 (mm, 가감속) --"));
  Serial.println(F("  MOVE <mm>            이송 (+정방향 / -역방향, 논블로킹)"));
  Serial.println(F("  STOP                 즉시 정지"));
  Serial.println(F("  GRIP OPEN|CLOSE|STOP|FREE|<mm>  cFS 별칭 (FREE=여자 해제)"));
  Serial.println(F("  -- 파라미터 (GCS 라이브 튜닝) --"));
  Serial.println(F("  PARAMS               전체 파라미터 + 범위"));
  Serial.println(F("  GETP [name]          값 조회 (생략=전체)"));
  Serial.println(F("  SETP <name> <val>    값 설정 (min/max clamp)"));
  Serial.println(F("  -- 홀센서 (위치 피드백) --"));
  Serial.println(F("  HALL                 홀 레벨/카운트 조회"));
  Serial.println(F("  HALLRST              카운트 0으로"));
  Serial.println(F("  HALLMON <0|1>        엣지 라이브 출력 on/off"));
  Serial.println(F("  (탈조감지: SETP stall_en 1 — 자석/스케일 검증 후 켤 것)"));
}

// STAT layout is a wire contract with cFS servicer_app, which does
// strstr("relays(1..8):") and strstr("bulldozer:") then looks for "IDLE" in the
// REMAINDER of a 160-byte buffer.  Two rules follow:
//   1. nothing new goes BEFORE the bulldozer line (truncation must only ever
//      eat the tail, never that line),
//   2. no line after it may contain the substring "IDLE" -- that would read as
//      "motion finished" while the bulldozer is still driving.
static void print_stat() {
  Serial.println(F("--- STATUS ---"));
  Serial.print(F("  relays(1..8): "));
  for (int i = 0; i < 8; i++) Serial.print((relay_state >> i) & 1);
  Serial.println();
  Serial.print(F("  bulldozer: "));
  Serial.print(mot_state == MOT_MOVING ? F("MOVING ") : F("IDLE"));
  if (mot_state == MOT_MOVING) {
    Serial.print(mot_dist_mm); Serial.print(F("mm "));
    Serial.print(mot_i); Serial.print(F("/")); Serial.print(mot_total);
    Serial.print(F(" d=")); Serial.print(mot_cur_delay); Serial.print(F("us"));
  } else if (mot_stop_why[0]) {
    Serial.print(F(" (")); Serial.print(mot_stop_why); Serial.print(F(")"));
  }
  Serial.println();
  Serial.print(F("  hall(PB10): lvl=")); Serial.print(digitalRead(PIN_HALL));
  Serial.print(F(" cnt="));              Serial.print(hall_count);
  Serial.print(F(" mm="));               Serial.print(mot_hall_mm(), 1);
  Serial.print(F(" lag="));              Serial.print(mot_cmd_mm() - mot_hall_mm(), 1);
  Serial.print(F(" st="));               Serial.print(mot_stalled);
  Serial.print(F(" gap="));              Serial.print(mot_gap_max);
  Serial.print(F(" rej="));              Serial.println(hall_rejected);
}

static void print_params() {
  Serial.print(F("--- PARAMS (")); Serial.print(PARAM_N); Serial.println(F(") ---"));
  for (int i = 0; i < PARAM_N; i++) {
    Serial.print(F("  ")); param_print(&PARAM_REG[i]);
    Serial.print(F("  [")); Serial.print(PARAM_REG[i].mn);
    Serial.print(F(", "));  Serial.print(PARAM_REG[i].mx); Serial.println(F("]"));
  }
}

// 라인 단위 명령 처리
static void parse_line(char* s) {
  char* cmd = strtok(s, " \t");
  if (!cmd) return;

  if      (!strcasecmp(cmd, "HELP")) { print_help(); }
  else if (!strcasecmp(cmd, "STAT")) { print_stat(); }
  else if (!strcasecmp(cmd, "BOOT")) {
    Serial.println(F(">> rebooting to ROM DFU..."));
    Serial.flush();
    reboot_to_bootloader();
  }
  else if (!strcasecmp(cmd, "RLY")) {
    char* a = strtok(NULL, " \t");
    char* b = strtok(NULL, " \t");
    if (!a || !b) { Serial.println(F("usage: RLY <1-8> <0|1>")); return; }
    int n = atoi(a), v = atoi(b);
    if (n < 1 || n > 8) { Serial.println(F("RLY: n=1..8")); return; }
    relay_write(n - 1, v != 0);
    Serial.print(F("RLY ")); Serial.print(n); Serial.print(F("=")); Serial.println(v ? 1 : 0);
  }
  else if (!strcasecmp(cmd, "RLYALL")) {
    char* a = strtok(NULL, " \t");
    if (!a) { Serial.println(F("usage: RLYALL <0-255>")); return; }
    uint8_t m = (uint8_t)atoi(a);
    relay_all(m);
    Serial.print(F("RLYALL=")); Serial.println(m);
  }
  else if (!strcasecmp(cmd, "MOVE")) {
    char* a = strtok(NULL, " \t");
    if (!a) { Serial.println(F("usage: MOVE <mm> (+정방향/-역방향)")); return; }
    motor_move(atof(a));
  }
  else if (!strcasecmp(cmd, "STOP")) {
    motor_stop("command");
  }
  // cFS servicer_app 호환 별칭 (불도저=그리퍼, 동일 모터): GRIP -> MOVE/STOP 라우팅
  else if (!strcasecmp(cmd, "GRIP")) {
    char* a = strtok(NULL, " \t");
    if (!a) { Serial.println(F("usage: GRIP OPEN|CLOSE|STOP|<mm>")); return; }
    if      (!strcasecmp(a, "OPEN"))  motor_move(mp.stroke_mm);     // +방향 stroke
    else if (!strcasecmp(a, "CLOSE")) motor_move(-mp.stroke_mm);    // -방향 stroke
    else if (!strcasecmp(a, "STOP"))  motor_stop("GRIP STOP");
    else if (!strcasecmp(a, "FREE")) {
      // Release the coils without moving: un-clamp after undock, cool-down, etc.
      if (mot_state == MOT_MOVING) { Serial.println(F("ERR FREE while moving")); return; }
      motor_enable(false);
      Serial.println(F("OK GRIP FREE (driver released)"));
    }
    else                              motor_move(atof(a));          // GRIP <mm> (부호 이송 = MOVE)
  }
  // 파라미터 라이브 튜닝 (GCS/cFS param_lib sync)
  else if (!strcasecmp(cmd, "PARAMS")) { print_params(); }
  else if (!strcasecmp(cmd, "GETP")) {
    char* a = strtok(NULL, " \t");
    if (!a) { print_params(); return; }
    const ParamReg* p = param_find(a);
    if (!p) { Serial.print(F("ERR GETP unknown: ")); Serial.println(a); return; }
    Serial.print(F("OK GETP ")); param_print(p); Serial.println();
  }
  else if (!strcasecmp(cmd, "SETP")) {
    char* a = strtok(NULL, " \t");
    char* b = strtok(NULL, " \t");
    if (!a || !b) { Serial.println(F("usage: SETP <name> <val>")); return; }
    const ParamReg* p = param_find(a);
    if (!p) { Serial.print(F("ERR SETP unknown: ")); Serial.println(a); return; }
    param_write(p, atof(b));
    Serial.print(F("OK SETP ")); param_print(p); Serial.println();
  }
  // 홀센서 (위치 피드백 / 탈조 검증)
  else if (!strcasecmp(cmd, "HALL")) {
    Serial.print(F("HALL lvl=")); Serial.print(digitalRead(PIN_HALL));
    Serial.print(F(" cnt="));     Serial.println(hall_count);
  }
  else if (!strcasecmp(cmd, "HALLRST")) {
    // Clear the move baseline with the counter, otherwise the next STAT reports
    // travel measured against a baseline that no longer exists.
    noInterrupts(); hall_count = 0; mot_hall0 = 0; hall_rejected = 0; interrupts();
    // Drop the commanded reference too (only while idle -- mot_i is live during
    // a move).  Without this, lag = last move's distance minus a freshly zeroed
    // measurement, so a reset between runs read as a huge shortfall.
    if (mot_state != MOT_MOVING) { mot_i = 0; mot_total = 0; }
    Serial.println(F("OK HALLRST cnt=0"));
  }
  else if (!strcasecmp(cmd, "HALLMON")) {
    char* a = strtok(NULL, " \t");
    hall_mon = (a && atoi(a) != 0);
    Serial.print(F("OK HALLMON ")); Serial.println(hall_mon ? 1 : 0);
  }
  else {
    Serial.print(F("unknown cmd: ")); Serial.println(cmd);
  }
}

// =====================================================================
//  setup / loop
// =====================================================================
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);                 // off (active-low)

  // 릴레이: push-pull 출력, 부팅 시 전부 close(OFF)
  for (int i = 0; i < 8; i++) {
    pinMode(RELAY_PINS[i], OUTPUT);
    relay_write(i, false);
  }

  // 불도저: STEP/DIR/EN push-pull (PNP/active-high), 부팅 시 idle + EN 비활성
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR,  OUTPUT);
  pinMode(PIN_EN,   OUTPUT);
  digitalWrite(PIN_STEP, LOW);                 // idle LOW (active-high)
  digitalWrite(PIN_DIR,  LOW);                 // 초기값 (MOVE 때 방향 설정)
  motor_enable(false);                         // 부팅 시 비활성 (안전)

  // 홀센서: 입력(모듈 자체 풀업) + FALLING 인터럽트로 자석 통과 카운트
  pinMode(PIN_HALL, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_HALL), hall_isr, FALLING);

  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 3000) {}

  Serial.println();
  Serial.println(F("Refuel Servicer 2 (BlackPill F411) — 밸브8 + 불도저 + 파라미터"));
  Serial.println(F("Relays OFF, Bulldozer IDLE. All safe at boot."));
  print_help();
  Serial.println(F("ready."));
}

void loop() {
  // 1) 불도저 논블로킹 스텝 (serial 폴링보다 가볍게, 매 loop)
  motor_update();

  // 1b) 홀 엣지 라이브 출력 (HALLMON 1일 때만; 기본 off라 cFS 파서 간섭 없음)
  static uint32_t hall_shown = 0;
  if (hall_mon && hall_count != hall_shown) {
    hall_shown = hall_count;
    Serial.print(F("HALL cnt=")); Serial.print(hall_count);
    Serial.print(F(" lvl="));     Serial.println(digitalRead(PIN_HALL));
  }

  // 2) 1 Hz LED
  static uint32_t t_led = 0;
  if (millis() - t_led >= 500) {
    t_led = millis();
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }

  // 3) serial 라인 수신 (논블로킹, blocking 금지)
  static char    buf[64];
  static uint8_t len = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (len > 0) { buf[len] = '\0'; parse_line(buf); len = 0; }
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }
}
