#include <IRremote.h> //IR 리모컨 라이브러리.
#define PIN_IR_RECEIVE 2
#define PIN_MOTOR_LEFT_FORWARD 10
#define PIN_MOTOR_LEFT_BACKWARD 9
#define PIN_MOTOR_RIGHT_FORWARD 6
#define PIN_MOTOR_RIGHT_BACKWARD 5
int motorSpeedLeft = 0; // -255~255
int motorSpeedRight = 0; // -255~255
void setup() {
  IrReceiver.begin(PIN_IR_RECEIVE, ENABLE_LED_FEEDBACK);
  pinMode(PIN_MOTOR_LEFT_FORWARD, OUTPUT);
  pinMode(PIN_MOTOR_LEFT_BACKWARD, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_FORWARD, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_BACKWARD, OUTPUT);
  analogWrite(PIN_MOTOR_LEFT_FORWARD, 0);
  analogWrite(PIN_MOTOR_LEFT_BACKWARD, 0);
  analogWrite(PIN_MOTOR_RIGHT_FORWARD, 0);
  analogWrite(PIN_MOTOR_RIGHT_BACKWARD, 0);
}

void loop() {
  if (IrReceiver.decode() == true) { // 리모컨으로부터 받은 신호가 있으면
    if (IrReceiver.decodedIRData.protocol == NEC) {
      if (IrReceiver.decodedIRData.command == 0x18) { // up
        motorSpeedLeft = 200;
        motorSpeedRight = 200;
      }
      else if (IrReceiver.decodedIRData.command == 0x52) { // down
        motorSpeedLeft = -200;
        motorSpeedRight = -200;
      }
      else if (IrReceiver.decodedIRData.command == 0x5A) { // right
        motorSpeedLeft = 200;
        motorSpeedRight = 150;
      }
      else if (IrReceiver.decodedIRData.command == 0x08) { // left
        motorSpeedLeft = 150;
        motorSpeedRight = 200;
      }
      else if (IrReceiver.decodedIRData.command == 0x1C) { // ok
        motorSpeedLeft = 0;
        motorSpeedRight = 0;
      }
    }
    IrReceiver.resume(); // 다음 신호 수신
  }
  if (motorSpeedLeft >= 0) {
    analogWrite(PIN_MOTOR_LEFT_FORWARD, motorSpeedLeft);
    analogWrite(PIN_MOTOR_LEFT_BACKWARD, 0);
  }
  else {
    analogWrite(PIN_MOTOR_LEFT_FORWARD, 0);
    analogWrite(PIN_MOTOR_LEFT_BACKWARD, motorSpeedLeft * (-1));
  }
  if (motorSpeedRight >= 0) {
    analogWrite(PIN_MOTOR_RIGHT_FORWARD, motorSpeedRight);
    analogWrite(PIN_MOTOR_RIGHT_BACKWARD, 0);
  }
  else {
    analogWrite(PIN_MOTOR_RIGHT_FORWARD, 0);
    analogWrite(PIN_MOTOR_RIGHT_BACKWARD, motorSpeedRight * (-1));
  }
}
