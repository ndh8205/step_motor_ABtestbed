#ifndef BULLDOZER_MOTOR_H
#define BULLDOZER_MOTOR_H

#include <Arduino.h>

// V1 (불도저 모드) — 단순 사다리꼴 프로파일. 동작은 원본 그대로, 구조만 분리.
//  800분주비 + 리드 2mm 기준 (STEPS_PER_MM = 800 / 2 = 400)
class BulldozerMotor {
  public:
    BulldozerMotor(int dirPin, int stepPin, int enPin);
    void begin();

    // 거리(mm) 이송. 부호로 방향 결정 (+ 정방향 / - 역방향)
    // 구동 중 시리얼 'p'/'P' 수신 시 긴급 정지
    void move(float distanceMm);

  private:
    int _dirPin;
    int _stepPin;
    int _enPin;

    // ── 튜닝 상수 (V1 값, STEPS_PER_MM만 400으로 보정) ──
    static const int  STEPS_PER_MM       = 400;  // 800분주비 ÷ 리드 2mm = 400 pulse/mm
    static const int  START_SPEED_DELAY  = 1100; // 출발 딜레이 [us] (초반 마찰 돌파)
    static const int  TARGET_SPEED_DELAY = 150;  // 최고속 딜레이 [us] (약 250 RPM 구간)
    static const long ACCEL_STEPS        = 1000; // 가속 구간 스텝 수

    // 한 방향 이송 (방향, 거리 절댓값)
    void _moveMotor(int direction, float distanceMm);

    // 긴급 정지 검사: 정지하면 true 반환
    bool _checkStop(unsigned long startTime);
};

#endif  // BULLDOZER_MOTOR_H