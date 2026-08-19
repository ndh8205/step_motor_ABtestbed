/*
 * GAIMSAT-1 Refuel CLIENT docking-port controller  (Arduino Uno R3)
 * -----------------------------------------------------------------
 * Driven over USB serial from jetson1 (/dev/gaimsat_client).  GCS refueling
 * dashboard -> cfs_bridge -> jetson1 arduino_client_bridge -> this sketch.
 *
 * I/O (actual bench wiring, 2026-07-20):
 *   - 4 client valve relays (active-LOW)   : relay 1,2,3,7 -> D2,D3,D4,D10
 *       (servicer valves 4,5,6 are on the servicer blackpill, NOT here)
 *   - client coupler stepper (SBD-20 + NK-266): DIR=D7 STEP=D6 EN=D5, COM=5V com.anode
 *       (active-LOW signals; non-blocking).  Motor A+/A-/B+/B- -> SBD-20; V+/GND = 24V.
 *   - 2 pressure sensors (DPH-100, 1..5V)  : A0 A1
 *   - status LED                           : D13
 *
 * ASCII line protocol (\n terminated, "OK ..."/"ERR ..."), 115200 8N1:
 *   VER / STAT / PRES
 *   RLY <1-8> <0|1> / RLYALL <0-255>
 *   STEP <revs>            move coupler (+CW / -CCW), non-blocking
 *   ENGAGE / RELEASE       coupler +/- MOVEREV revolutions (presets)
 *   SCHED ENGAGE|RELEASE <sec> / SCHED OFF
 *                          one-shot deferred move, fires <sec> seconds later.
 *                          The command link runs THROUGH the pogo pins, so once
 *                          the servicer undocks nobody can command this board;
 *                          the refuel sequence arms this while still docked and
 *                          the coupler restores itself after separation.
 *                          ENGAGE also self-arms this (see AUTORET below): every
 *                          engage is followed by an automatic -MOVEREV restore
 *                          AUTORET seconds later unless RELEASE/STEPSTOP/SCHED
 *                          intervenes.  AUTORET=0 turns that off.
 *   STEPSTOP               halt move + release driver (also cancels SCHED)
 *   GETP                   dump stepper params
 *   SETP <name> <value>    SPEED(us) | SPR(steps/rev) | DIRINV(0/1) | ENPOL(0/1) | MOVEREV(rev)
 *                          ENPOL: 1 = EN active HIGH (com.anode), 0 = active LOW.
 */

const int8_t RELAY_PIN[9] = { -1, 2, 3, 4, -1, -1, -1, 10, -1 };   // relay# -> pin; -1 = not here
const uint8_t DIR_PIN  = 7;
const uint8_t STEP_PIN = 6;
const uint8_t EN_PIN   = 5;
const uint8_t PRES_PIN[2] = { A0, A1 };
const uint8_t LED_PIN  = 13;

// --- GCS-tunable stepper params (runtime, no re-flash) ---
unsigned int g_speed_us  = 400;    // SPEED: STEP pulse half-period [us]
long         g_steps_rev = 1600;   // SPR:   steps per revolution
bool         g_dir_inv   = false;  // DIRINV: flip direction
bool         g_en_high   = true;   // ENPOL: true = EN active HIGH (com.anode)
float        g_move_rev  = 2.0f;   // MOVEREV: ENGAGE/RELEASE preset revolutions
// HOLD: keep the driver energised when idle.  The coupler sits in a load path
// that back-drives the screw when the coils are released (observed on the rig
// 2026-08-11: engaged coupler pushed back under load), so holding torque is the
// default.  Cost: continuous coil current from the 24 V motor battery + heat --
// turn it off for long idle storage.
bool         g_hold      = true;
// AUTORET: every ENGAGE auto-arms the deferred restore (-MOVEREV), counted
// from the moment the engage MOVE COMPLETES -- not from the command.  The +9
// rev move itself takes ~11 s, so counting from the command would make short
// delays fire mid-move and restore with zero dwell.  Manual RELEASE or
// STEPSTOP cancels it; 0 disables.
long         g_autoret_s = 5;

static uint8_t enActive()   { return g_en_high ? HIGH : LOW; }
static uint8_t enInactive() { return g_en_high ? LOW : HIGH; }

// --- stepper state (non-blocking) ---
long          g_stepsRemaining = 0;
bool          g_stepLow = false;
unsigned long g_lastStepUs = 0;

// --- deferred one-shot move (armed while docked, fires after link loss) ---
int8_t        g_schedDir  = 0;      // 0 = none, +1 = ENGAGE, -1 = RELEASE
unsigned long g_schedAtMs = 0;      // millis() at which to fire
bool          g_autoretPend = false; // ENGAGE done -> start AUTORET when move ends

char   g_buf[56];
uint8_t g_len = 0;

static int8_t relayPin(uint8_t n) { return (n >= 1 && n <= 8) ? RELAY_PIN[n] : -1; }
static void relayWrite(uint8_t n, bool on) { int8_t p = relayPin(n); if (p >= 0) digitalWrite(p, on ? LOW : HIGH); }

void setup() {
  for (uint8_t n = 1; n <= 8; n++) { int8_t p = relayPin(n); if (p >= 0) { pinMode(p, OUTPUT); digitalWrite(p, HIGH); } }
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT); digitalWrite(STEP_PIN, HIGH);   // idle = opto OFF (com.anode)
  pinMode(EN_PIN, OUTPUT);   digitalWrite(EN_PIN, enInactive());
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
}

static void serviceStepper() {
  if (g_stepsRemaining <= 0) return;
  digitalWrite(LED_PIN, HIGH);
  unsigned long now = micros();
  if (now - g_lastStepUs < g_speed_us) return;
  g_lastStepUs = now;
  if (!g_stepLow) { digitalWrite(STEP_PIN, LOW); g_stepLow = true; }        // assert pulse
  else {
    digitalWrite(STEP_PIN, HIGH); g_stepLow = false;                        // release -> one step
    if (--g_stepsRemaining == 0) {
      if (!g_hold) digitalWrite(EN_PIN, enInactive());   // hold: keep torque on
      digitalWrite(LED_PIN, LOW);
      if (g_autoretPend) {               // engage finished: NOW start the restore countdown
        g_autoretPend = false;
        g_schedDir  = -1;
        g_schedAtMs = millis() + (unsigned long)g_autoret_s * 1000UL;
      }
    }
  }
}

static void startMove(float revs) {
  digitalWrite(EN_PIN, enActive());
  bool cw = (revs >= 0) ^ g_dir_inv;
  digitalWrite(DIR_PIN, cw ? HIGH : LOW);
  g_stepsRemaining = (long)(fabs(revs) * g_steps_rev + 0.5);
  g_stepLow = false;
}

// Fire a pending deferred move once its time arrives.  Signed diff is
// rollover-safe; skipped while a move is in progress so the deferred move
// starts from rest (retried next loop, not lost).
static void serviceSchedule() {
  if (g_schedDir == 0 || g_stepsRemaining > 0) return;
  if ((long)(millis() - g_schedAtMs) < 0) return;
  startMove(g_schedDir > 0 ? +g_move_rev : -g_move_rev);
  g_schedDir = 0;
}

static void reportStat() {
  char r[9];
  for (uint8_t n = 1; n <= 8; n++) { int8_t p = relayPin(n); r[n - 1] = (p >= 0 && digitalRead(p) == LOW) ? '1' : '0'; }
  r[8] = 0;
  Serial.print(F("OK STAT relays=")); Serial.print(r);
  Serial.print(F(" step=")); Serial.print(g_stepsRemaining > 0 ? F("moving") : F("idle"));
  Serial.print(F(" rem=")); Serial.print(g_stepsRemaining);
  Serial.print(F(" p1=")); Serial.print(analogRead(PRES_PIN[0]));
  Serial.print(F(" p2=")); Serial.print(analogRead(PRES_PIN[1]));
  // sched sits after p2 so the jetson bridge regex (search, not match) still hits
  Serial.print(F(" sched="));
  if (g_autoretPend) Serial.println(F("REL:move"));      // countdown starts when the move ends
  else if (g_schedDir == 0) Serial.println(F("none"));
  else {
    long left = (long)(g_schedAtMs - millis()); if (left < 0) left = 0;
    Serial.print(g_schedDir > 0 ? F("ENG:") : F("REL:")); Serial.println(left / 1000);
  }
}

static void reportParams() {
  Serial.print(F("OK PARAMS speed=")); Serial.print(g_speed_us);
  Serial.print(F(" spr=")); Serial.print(g_steps_rev);
  Serial.print(F(" dirinv=")); Serial.print(g_dir_inv ? 1 : 0);
  Serial.print(F(" enpol=")); Serial.print(g_en_high ? 1 : 0);
  Serial.print(F(" moverev=")); Serial.print(g_move_rev, 2);
  Serial.print(F(" autoret=")); Serial.print(g_autoret_s);
  Serial.print(F(" hold=")); Serial.println(g_hold ? 1 : 0);
}

static void setParam(char *s) {                 // s = "<name> <value>"
  char *sp = strchr(s, ' ');
  if (!sp) { Serial.println(F("ERR SETP syntax")); return; }
  *sp = 0; char *val = sp + 1;
  if      (!strcmp(s, "SPEED"))   { long v = atol(val); if (v < 40 || v > 20000) { Serial.println(F("ERR SPEED range")); return; } g_speed_us = v; }
  else if (!strcmp(s, "SPR"))     { long v = atol(val); if (v < 1 || v > 100000) { Serial.println(F("ERR SPR range")); return; } g_steps_rev = v; }
  else if (!strcmp(s, "DIRINV"))  { g_dir_inv = atoi(val) != 0; }
  else if (!strcmp(s, "ENPOL"))   { g_en_high = atoi(val) != 0; if (g_stepsRemaining <= 0) digitalWrite(EN_PIN, g_hold ? enActive() : enInactive()); }
  else if (!strcmp(s, "HOLD"))    { g_hold = atoi(val) != 0;
    // Takes effect NOW, not at the next move: on = clamp the shaft immediately
    // (zero motion), off = free it.
    if (g_stepsRemaining <= 0) digitalWrite(EN_PIN, g_hold ? enActive() : enInactive()); }
  else if (!strcmp(s, "MOVEREV")) { g_move_rev = atof(val); }
  else if (!strcmp(s, "AUTORET")) { long v = atol(val); if (v < 0 || v > 3600) { Serial.println(F("ERR AUTORET range")); return; } g_autoret_s = v; }
  else { Serial.print(F("ERR SETP unknown ")); Serial.println(s); return; }
  reportParams();
}

static void handleLine(char *s) {
  if      (!strcmp(s, "VER"))  Serial.println(F("OK VER CLIENT-DOCK 1.6 RLY(1,2,3,7) STEP(SBD20 D7/D6/D5) SCHED AUTORET HOLD PARAMS PRES(A0,A1)"));
  else if (!strcmp(s, "STAT")) reportStat();
  else if (!strcmp(s, "GETP")) reportParams();
  else if (!strcmp(s, "PRES")) {
    Serial.print(F("OK PRES"));
    for (uint8_t i = 0; i < 2; i++) { int a = analogRead(PRES_PIN[i]); long mV = (long)a * 5000L / 1023L;
      Serial.print(F(" p")); Serial.print(i + 1); Serial.print('='); Serial.print(a); Serial.print(' '); Serial.print(mV); Serial.print(F("mV")); }
    Serial.println();
  }
  else if (!strncmp(s, "SETP ", 5)) setParam(s + 5);
  else if (!strncmp(s, "RLYALL ", 7)) { int m = atoi(s + 7); if (m < 0 || m > 255) { Serial.println(F("ERR RLYALL range")); return; }
    for (uint8_t n = 1; n <= 8; n++) relayWrite(n, (m >> (n - 1)) & 1); Serial.print(F("OK RLYALL=")); Serial.println(m); }
  else if (!strncmp(s, "RLY ", 4)) { char *p = s + 4; int n = atoi(p); char *sp = strchr(p, ' '); int v = sp ? atoi(sp + 1) : 0;
    if (n < 1 || n > 8) { Serial.println(F("ERR RLY num")); return; }
    if (relayPin(n) < 0) { Serial.print(F("ERR RLY ")); Serial.print(n); Serial.println(F(" not on this board")); return; }
    relayWrite(n, v != 0); Serial.print(F("OK RLY ")); Serial.print(n); Serial.print('='); Serial.println(v ? 1 : 0); }
  else if (!strncmp(s, "STEP ", 5)) { float r = atof(s + 5); startMove(r); Serial.print(F("OK STEP ")); Serial.println(r, 3); }
  else if (!strcmp(s, "ENGAGE"))  {
    startMove(+g_move_rev);
    // Engage never stays engaged: the restore countdown starts when this move
    // completes (see serviceStepper), so AUTORET is real dwell time.
    g_autoretPend = (g_autoret_s > 0);
    g_schedDir = 0;
    if (g_autoretPend && g_stepsRemaining == 0) {
      // Zero-length engage never reaches the completion branch, which would
      // leave the pending flag stuck until some unrelated move finished.
      g_autoretPend = false;
      g_schedDir  = -1;
      g_schedAtMs = millis() + (unsigned long)g_autoret_s * 1000UL;
    }
    Serial.print(F("OK ENGAGE ")); Serial.print(g_move_rev, 2);
    if (g_autoret_s > 0) { Serial.print(F(" autoret=")); Serial.print(g_autoret_s); Serial.println(F("s after move")); }
    else Serial.println();
  }
  else if (!strcmp(s, "RELEASE")) {
    g_schedDir = 0; g_autoretPend = false;   // manual restore replaces the automatic one
    startMove(-g_move_rev); Serial.print(F("OK RELEASE ")); Serial.println(g_move_rev, 2);
  }
  else if (!strncmp(s, "SCHED ", 6)) {
    char *a = s + 6;
    if (!strcmp(a, "OFF")) { g_schedDir = 0; g_autoretPend = false; Serial.println(F("OK SCHED OFF")); }
    else {
      char *sp = strchr(a, ' ');
      long sec = sp ? atol(sp + 1) : 0;
      if (sp) *sp = 0;
      if (sec < 1 || sec > 3600) { Serial.println(F("ERR SCHED sec 1..3600")); }
      else if (!strcmp(a, "ENGAGE") || !strcmp(a, "RELEASE")) {
        g_schedDir  = (a[0] == 'E') ? +1 : -1;
        g_schedAtMs = millis() + (unsigned long)sec * 1000UL;
        Serial.print(F("OK SCHED ")); Serial.print(a); Serial.print(' '); Serial.print(sec); Serial.println(F("s"));
      }
      else Serial.println(F("ERR SCHED verb"));
    }
  }
  else if (!strcmp(s, "STEPSTOP")) { g_stepsRemaining = 0; g_schedDir = 0; g_autoretPend = false; digitalWrite(STEP_PIN, HIGH); digitalWrite(EN_PIN, enInactive()); Serial.println(F("OK STEPSTOP")); }
  else if (s[0] == 0) { /* ignore */ }
  else { Serial.print(F("ERR unknown_cmd ")); Serial.println(s); }
}

void loop() {
  serviceStepper();
  serviceSchedule();
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') { g_buf[g_len] = 0; handleLine(g_buf); g_len = 0; }
    else if (g_len < sizeof(g_buf) - 1) g_buf[g_len++] = c;
    serviceStepper();
  }
}
