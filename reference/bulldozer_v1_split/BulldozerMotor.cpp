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

// ── 외부 호출: 부호로 방향 결정 후 이송 ──
void BulldozerMotor::move(float distanceMm) {
  if (distanceMm > 0) {
    Serial.print(">> 정방향 불도저 이송 시작: ");
    Serial.print(distanceMm);
    Serial.println(" mm");
    _moveMotor(HIGH, distanceMm);
  }
  else if (distanceMm < 0) {
    Serial.print(">> 역방향 불도저 이송 시작: ");
    Serial.print(-distanceMm);
    Serial.println(" mm");
    _moveMotor(LOW, -distanceMm);
  }
}

// ── 내부: 한 방향 사다리꼴 이송 (원본 V1 로직 그대로) ──
void BulldozerMotor::_moveMotor(int direction, float distanceMm) {
  digitalWrite(_dirPin, direction);
  long totalSteps = distanceMm * STEPS_PER_MM;
  long currentAccelSteps = ACCEL_STEPS;

  if (totalSteps < currentAccelSteps * 2) {
    currentAccelSteps = totalSteps / 2;
  }

  unsigned long startTime = millis();

  for (long i = 0; i < totalSteps; i++) {
    // 8스텝마다 긴급 정지 검사 (연산 최적화 유지)
    if ((i & 7) == 0) {
      if (_checkStop(startTime)) return;
    }

    int currentDelay = TARGET_SPEED_DELAY;
    if (i < currentAccelSteps) {
      currentDelay = map(i, 0, currentAccelSteps, START_SPEED_DELAY, TARGET_SPEED_DELAY);
    }
    else if (i > totalSteps - currentAccelSteps) {
      currentDelay = map(i, totalSteps - currentAccelSteps, totalSteps, TARGET_SPEED_DELAY, START_SPEED_DELAY);
    }

    digitalWrite(_stepPin, HIGH);
    delayMicroseconds(currentDelay);
    digitalWrite(_stepPin, LOW);
    delayMicroseconds(currentDelay);
  }

  unsigned long endTime = millis();
  float actualRunTime = (endTime - startTime) / 1000.0;
  Serial.print(">> 이송 완료. (이동 거리: ");
  Serial.print(distanceMm);
  Serial.print(" mm / 최종 소요 시간: ");
  Serial.print(actualRunTime, 2);
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