#include "BulldozerMotor.h"

// 핀 배선: dir = 7, step = 6, en = 5
BulldozerMotor motor(7, 6, 5);

void setup() {
  motor.begin();

  Serial.begin(9600);
  Serial.println("== [BULLDOZER MODE] High-Torque Ball Screw Ready ==");
  Serial.println("이동할 거리(mm)를 입력하세요. (예: 60, -30 등)");
  Serial.println("★★ 구동 중 'p'를 입력하면 긴급 정지합니다. ★★\n");
}

void loop() {
  if (Serial.available() > 0) {
    char checkChar = Serial.peek();

    // 정지문자/개행은 버림
    if (checkChar == 'p' || checkChar == 'P' || checkChar == '\n' || checkChar == '\r') {
      Serial.read();
      return;
    }

    float targetDistanceMm = Serial.parseFloat();
    if (targetDistanceMm != 0.0) {
      motor.move(targetDistanceMm);  // 부호로 방향 자동 결정
    }
  }
}
