# step_motor_ABtestbed

A·B 두 테스트베드의 스텝모터 제어 펌웨어다. 리그에 실제로 플래시해서 돌리고 있는 소스를 그대로 올렸다.

| | A — 서비서 | B — 클라이언트 |
|---|---|---|
| 보드 | WeAct Black Pill V3.x (STM32F411CEU6) | Arduino Uno R3 |
| 구동부 | 볼스크류 직선이송 (불도저) | 커플러 회전 |
| 드라이버 / 모터 | SBD-20 / NK266 | SBD-20 / NK-266 |
| 명령 단위 | mm | 회전수(rev) |
| 위치 피드백 | 홀센서 A3144E (자석 2개) | 없음 |
| 호스트 링크 | USB CDC 115200 | USB 시리얼 115200 |
| 소스 | `A_servicer_blackpill_f411/src/main.cpp` | `B_client_arduino_uno/client_dock.ino` |

두 보드 모두 스텝 펄스를 **논블로킹**으로 만든다. `loop()`에서 half-delay마다 STEP 핀을 토글할 뿐이고, `delay()`로 막지 않는다. 그래서 이동 중에도 시리얼 명령을 계속 받는다.

## ⚠️ 같은 드라이버, 반대 극성

SBD-20은 COM 단자를 어디에 묶느냐에 따라 신호 극성이 뒤집힌다. A와 B가 서로 다르게 물려 있으니 코드를 옮겨 쓸 때 반드시 확인할 것.

| | A (서비서) | B (클라이언트) |
|---|---|---|
| COM 결선 | **GND** (PNP / com.cathode) | **5V** (com.anode) |
| STEP | idle LOW, **HIGH가 펄스** | idle HIGH, **LOW가 펄스** |
| EN | **LOW = 활성** (실측) | **HIGH = 활성** (`ENPOL=1`) |
| GPIO | push-pull | push-pull |

A의 EN 극성은 벤치 실측으로 확정했다. 데이터시트 설명("옵토 ON = disable")과 반대로 읽히니 주의.

## A — 서비서 블랙필

### 핀맵
| 기능 | 핀 |
|---|---|
| STEP | PB6 |
| DIR | PB7 |
| EN | PB8 |
| 홀센서 DO | PB10 (FT, FALLING 인터럽트) |
| 릴레이 8 (밸브) | PB12 PB13 PB14 PB15 PA8 PA9 PA10 PB5 |
| LED | PC13 (active-low) |

### 제어 구조
- `motor_move(mm)` → 스텝 수 계산, DIR 세팅, EN 활성, 상태만 잡고 즉시 반환.
- `motor_update()` → `loop()`마다 호출. half-delay 경과 시 STEP 토글, HIGH→LOW에서 1스텝 카운트.
- 가감속은 `mot_delay_at(i)`가 계산한다. 선형 보간이 기본이고 `accel_profile=1`이면 지수 보간이다. 출발 1100 µs → 최고속 150 µs, 가감속 구간 1000스텝.
- `steps_per_mm = 400` (800분주 ÷ 리드 2 mm), 행정 `stroke_mm = 65`.
- 홀딩 정책 `hold`: 0 = 안 잡음, 1 = 항상, **2 = 닫는 이동(음의 이송) 뒤에만 유지**. 2가 운용 기본값이다. 열기·시험 이동 뒤에는 여자를 풀어 발열을 줄인다.

### 홀센서와 탈조 감지
자석 2개가 축 원주면에 180도로 붙어 있어 1회전에 2펄스가 나온다. 벤치 실측으로 `hall_per_mm = 2.0`을 확정했다(60 mm 왕복에 119–120펄스).

노이즈가 실제로 섞여 들어오므로 ISR에 간격 필터를 뒀다. 고정 문턱 하나로는 못 막는다 — 진짜 펄스 간격이 램프 초반 ~440 ms에서 최고속 ~60 ms까지 변하기 때문이다. 그래서 `hall_min_interval()`이 **현재 스텝 주기에서 최소 간격을 역산**하고, 그보다 빨리 들어온 엣지를 버린다(`hall_rejected`로 집계). 여유값 `hall_dyn_marg = 0.5`가 실측 최적이다. 0.75로 조이면 진짜 펄스까지 버린다.

탈조 가드(`stall_en`)는 명령 이동량과 홀 실측을 비교해 `stall_mm` 이상 벌어지면 이동을 멈춘다. **기본값은 꺼짐**이다. 자석 장착과 스케일 검증을 마친 뒤에 켤 것. 홀이 안 붙은 상태에서 켜면 정상 이동도 탈조로 오판한다.

> 홀은 단채널이라 방향을 모른다. 이동량의 크기만 알 수 있고, 오버슈트는 감지 못 한다.

### 명령 (ASCII, 개행 종료)
```
MOVE <mm>              이송 (+정방향 / -역방향)
STOP                   즉시 정지
GRIP OPEN|CLOSE|STOP|FREE|<mm>   행정 단위 별칭 (FREE = 이동 없이 여자 해제)
PARAMS / GETP [name] / SETP <name> <val>    파라미터 라이브 튜닝 (재플래시 없음)
HALL / HALLRST / HALLMON <0|1>              홀 조회·리셋·라이브 출력
STAT / HELP / BOOT     상태 / 도움말 / ROM DFU 재진입
RLY <1-8> <0|1> / RLYALL <0-255>            릴레이(밸브)
```
파라미터 15개는 `PARAM_REG[]` 한 줄로 등록한다. 값은 RAM에 있어 재부팅하면 기본값으로 돌아간다.

### 빌드·플래시
```bash
pio run -d A_servicer_blackpill_f411                 # 빌드
pio run -d A_servicer_blackpill_f411 -t upload       # ROM DFU 업로드
```
첫 플래시만 BOOT0 + RESET으로 ROM DFU에 들어간다. 그 뒤로는 시리얼에 `BOOT`를 보내면 펌웨어가 스스로 DFU로 점프한다 — 버튼 없이 재플래시된다.

> `reboot_to_bootloader()`는 손대지 말 것. 클럭·USB 초기화 순서가 검증된 시퀀스다.

## B — 클라이언트 아두이노

### 핀맵
| 기능 | 핀 |
|---|---|
| DIR | D7 |
| STEP | D6 |
| EN | D5 |
| COM | 5V |
| 릴레이 (밸브 1·2·3·7) | D2 D3 D4 D10 |
| 압력센서 2 (DPH-100, 1–5 V) | A0 A1 |
| LED | D13 (스텝 구동 중 점등) |

배선 상세는 [`B_client_arduino_uno/WIRING.md`](B_client_arduino_uno/WIRING.md)에 있다. 공통 접지, 릴레이 JD-VCC 분리, 압력센서 4선 색상 주의사항까지 적어놨다.

### 제어 구조
- `startMove(revs)` → EN 활성, DIR 세팅, 남은 스텝 수만 채우고 반환.
- `serviceStepper()` → `loop()`과 시리얼 수신 루프 **양쪽에서** 호출한다. 긴 명령이 들어와도 펄스 열이 끊기지 않는다.
- 기본값: half-period 400 µs, 1600 step/rev, 프리셋 이동 `MOVEREV = 2.0` rev.
- `HOLD` 기본 켜짐. 커플러가 하중을 받는 자리에 있어서 여자를 풀면 스크류가 역구동된다(리그 실측). 장기 보관 때만 끌 것.
- `AUTORET` — ENGAGE는 항상 자동 복귀를 건다. **카운트다운은 명령 시점이 아니라 이동 완료 시점부터** 시작한다. 이동이 길면 십수 초가 걸린다(9 rev ≈ 11초). 명령 시점부터 세면 짧은 지연값이 이동 도중에 발화한다.
- `SCHED ENGAGE|RELEASE <sec>` — 예약 이동. 명령 링크가 포고핀을 지나가는 구조라 분리 후에는 이 보드에 명령을 못 보낸다. 붙어 있는 동안 예약을 걸어두면 분리 뒤 커플러가 알아서 복귀한다.

### 명령
```
STEP <revs>            회전 이송 (+CW / -CCW)
ENGAGE / RELEASE       ±MOVEREV 프리셋
SCHED ENGAGE|RELEASE <sec> / SCHED OFF      예약 1회 이동
STEPSTOP               정지 + 여자 해제 (예약도 취소)
GETP / SETP <name> <val>    SPEED SPR DIRINV ENPOL MOVEREV AUTORET HOLD
VER / STAT / PRES      버전 / 상태 / 압력 판독
RLY <1-8> <0|1> / RLYALL <0-255>
```
응답은 전부 `OK ...` 또는 `ERR ...` 한 줄이다.

### 빌드·플래시
우노는 DFU가 없다. 빌드해서 hex를 뽑고 avrdude로 굽는다.
```bash
pio run -d B_client_arduino_uno                      # -> .pio/build/uno/firmware.hex
avrdude -c arduino -p atmega328p -P <포트> -b 115200 -U flash:w:firmware.hex:i   # 예: /dev/ttyACM1
```
> 우노는 시리얼을 열 때 DTR로 리셋된다. 포트 open 후 2초쯤 기다렸다가 명령을 보낼 것.

## 참고

- 두 소스 모두 **원본 그대로**다. 스텝 제어 외에 릴레이(밸브)와 압력센서 코드가 같이 들어 있다. 스텝만 볼 거면 A는 `motor_*` / `hall_*` 함수, B는 `serviceStepper` / `startMove` / `serviceSchedule`을 보면 된다.
- A의 `STAT` 출력 형식은 상위 소프트웨어와의 계약이다. `bulldozer:` 줄 앞에 새 줄을 넣지 말고, 그 뒤 어느 줄에도 `IDLE` 문자열을 넣지 말 것. 소스 주석에 이유를 적어놨다.
- A에 펄스 폭 검사를 넣었다가 되돌린 적이 있다. 진짜 펄스를 스파이크로 오판해 이동량 판독이 나빠졌다. 스코프로 실파형을 보기 전에는 다시 넣지 말 것.
