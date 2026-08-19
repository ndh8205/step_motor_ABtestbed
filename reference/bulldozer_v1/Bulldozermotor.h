#ifndef BULLDOZER_MOTOR_H
#define BULLDOZER_MOTOR_H

#include <Arduino.h>

// SBD-20 드라이버 + NK-266 스텝모터(불도저 모드) 제어 클래스
class BulldozerMotor {
  public:
    // 핀 번호를 받아 객체 생성 (dir, step, en 순서)
    BulldozerMotor(int dirPin, int stepPin, int enPin);

    // pinMode 설정 및 드라이버 활성화 (setup()에서 1회 호출)
    void begin();

    // 거리(mm) 이송. 부호로 방향 결정 (+ 정방향 / - 역방향)
    // 구동 중 시리얼로 'p'/'P' 수신 시 긴급 정지
    void move(float distanceMm);

  private:
    int _dirPin;
    int _stepPin;
    int _enPin;

    // ── 튜닝 파라미터 (불도저 모드) ──
    static const int  STEPS_PER_MM = 200;   // 1/4 분주(800 P/rev), 리드 4mm 기준
    static const int  START_DELAY  = 1100;  // 출발 딜레이(us): 초반 정지마찰 돌파
    static const int  TARGET_DELAY = 150;   // 최고속 딜레이(us): 약 250 RPM 구간
    static const long ACCEL_STEPS  = 1000;  // 가·감속 구간 스텝 수

    // 긴급 정지 검사: 정지하면 true 반환
    bool _checkStop(unsigned long startTime);
};

#endif  // BULLDOZER_MOTOR_H
