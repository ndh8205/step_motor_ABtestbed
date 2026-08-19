#include "BulldozerMotor.h"

// ── 생성자: 핀 번호 저장 ──
BulldozerMotor::BulldozerMotor(int dirPin, int stepPin, int enPin) {
  _dirPin  = dirPin;
  _stepPin = stepPin;
  _enPin   = enPin;
}

// ── 초기화: 핀 모드 + 드라이버 활성화 ──
void BulldozerMotor::begin() {
  pinMode(_stepPin, OUTPUT);
  pinMode(_dirPin,  OUTPUT);
  pinMode(_enPin,   OUTPUT);
  digitalWrite(_enPin, HIGH);  // 모터 드라이버 활성화
}

// ── 이송 본체 ──
void BulldozerMotor::move(float distanceMm) {
  // 부호로 방향 결정, 거리는 절댓값으로
  int   direction = (distanceMm >= 0) ? HIGH : LOW;
  float absDist   = (distanceMm >= 0) ? distanceMm : -distanceMm;
  digitalWrite(_dirPin, direction);

  long totalSteps = (long)(absDist * STEPS_PER_MM);

  // 짧은 이동이면 가속 구간을 절반으로 축소 (사다리꼴 유지)
  long currentAccelSteps = ACCEL_STEPS;
  if (totalSteps < currentAccelSteps * 2) {
    currentAccelSteps = totalSteps / 2;
  }

  unsigned long startTime = millis();

  for (long i = 0; i < totalSteps; i++) {
    // 8스텝마다 긴급 정지 폴링 (연산 최적화)
    if ((i & 7) == 0) {
      if (_checkStop(startTime)) return;
    }

    int currentDelay = TARGET_DELAY;
    if (i < currentAccelSteps) {
      currentDelay = map(i, 0, currentAccelSteps, START_DELAY, TARGET_DELAY);
    } else if (i > totalSteps - currentAccelSteps) {
      currentDelay = map(i, totalSteps - currentAccelSteps, totalSteps, TARGET_DELAY, START_DELAY);
    }

    digitalWrite(_stepPin, HIGH);
    delayMicroseconds(currentDelay);
    digitalWrite(_stepPin, LOW);
    delayMicroseconds(currentDelay);
  }

  float runTime = (millis() - startTime) / 1000.0;
  Serial.print(">> 이송 완료. (최종 소요 시간: ");
  Serial.print(runTime, 2);
  Serial.println(" 초)\n");
}

// ── 긴급 정지 검사 ──
bool BulldozerMotor::_checkStop(unsigned long startTime) {
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'p' || c == 'P') {
      float stoppedRunTime = (millis() - startTime) / 1000.0;
      Serial.print("\n[!] 긴급 정지. (실행 시간: ");
      Serial.print(stoppedRunTime, 2);
      Serial.println(" 초)");
      return true;
    }
  }
  return false;
}