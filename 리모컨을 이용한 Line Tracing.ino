#include <IRremote.h> //IR 리모컨 라이브러리.

#define PIN_IR_RECEIVE 2
#define PIN_MOTOR_LEFT_FORWARD 10
#define PIN_MOTOR_LEFT_BACKWARD 9
#define PIN_MOTOR_RIGHT_FORWARD 6
#define PIN_MOTOR_RIGHT_BACKWARD 5
#define PIN_LINE_DETECT_RIGHT A0
#define PIN_LINE_DETECT_LEFT  A1
#define BASE_SPEED 100   
#define REF_VALUE_LINE_DETECT  850
#define TURN_SPEED 80   
#define KP  2.733
#define KI  0.0
#define KD  1.911
int motorSpeedLeft = 0; // -255~255
int motorSpeedRight = 0; // -255~255
int isStartLineTracing = 0;
int detectValueLeft = 0;
int detectValueRight = 0;





int baseSpeed = 120;  // 기본 모터 속도

void processMotor(void);

void setup()
{
  Serial.begin(115200); //시리얼 모니터 사용
  IrReceiver.begin(PIN_IR_RECEIVE, ENABLE_LED_FEEDBACK);
  pinMode(PIN_MOTOR_LEFT_FORWARD, OUTPUT);
  pinMode(PIN_MOTOR_LEFT_BACKWARD, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_FORWARD, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_BACKWARD, OUTPUT);
  analogWrite(PIN_MOTOR_LEFT_FORWARD, 0);
  analogWrite(PIN_MOTOR_LEFT_BACKWARD, 0);
  analogWrite(PIN_MOTOR_RIGHT_FORWARD, 0);
  analogWrite(PIN_MOTOR_RIGHT_BACKWARD, 0);
  Serial.println("0003");
}

void loop() {
  if (IrReceiver.decode() == true) // 리모컨으로부터 받은 신호가 있으면
  {
    if (IrReceiver.decodedIRData.protocol == NEC) {
      Serial.println(IrReceiver.decodedIRData.command, HEX); //받은 신호를 시리얼 모니터에 입력
      if (IrReceiver.decodedIRData.command == 0x16) // *
      {
        Serial.println("start");
        isStartLineTracing = 1;
      }
      else if (IrReceiver.decodedIRData.command == 0x0D) // #
      {
        Serial.println("stop");
        isStartLineTracing = 0;
      }
    }
    IrReceiver.resume(); // 다음 신호 수신
  }

  if (isStartLineTracing == 0)
  {
    motorSpeedLeft = 0;
    motorSpeedRight = 0;
    processMotor();
  }
  else
  {
    //detectValueLeft = analogRead(PIN_LINE_DETECT_LEFT);
    detectValueRight = analogRead(PIN_LINE_DETECT_RIGHT);
    if (detectValueRight > REF_VALUE_LINE_DETECT + 50) {
      // 센서가 기준 값보다 크면 → 오른쪽으로 치우침 → 왼쪽으로 회전
      motorSpeedLeft = BASE_SPEED - TURN_SPEED;
      motorSpeedRight = BASE_SPEED + TURN_SPEED;
    } else if (detectValueRight < REF_VALUE_LINE_DETECT - 50) {
      // 센서가 기준 값보다 작으면 → 왼쪽으로 치우침 → 오른쪽으로 회전
      motorSpeedLeft = BASE_SPEED - TURN_SPEED;
      motorSpeedRight = BASE_SPEED + TURN_SPEED;
    } else {
      // 센서가 기준 값 근처이면 → 직진
      motorSpeedLeft = BASE_SPEED;
      motorSpeedRight = BASE_SPEED + 17;
    }

    processMotor(); // 모터 제어
  }
}

void processMotor(void)
{
  if (motorSpeedLeft >= 0)
  {
    analogWrite(PIN_MOTOR_LEFT_FORWARD, motorSpeedLeft);
    analogWrite(PIN_MOTOR_LEFT_BACKWARD, 0);
  }
  else
  {
    analogWrite(PIN_MOTOR_LEFT_FORWARD, 0);
    analogWrite(PIN_MOTOR_LEFT_BACKWARD, motorSpeedLeft * (-1));
  }
  if (motorSpeedRight >= 0)
  {
    analogWrite(PIN_MOTOR_RIGHT_FORWARD, motorSpeedRight);
    analogWrite(PIN_MOTOR_RIGHT_BACKWARD, 0);
  }
  else
  {
    analogWrite(PIN_MOTOR_RIGHT_FORWARD, 0);
    analogWrite(PIN_MOTOR_RIGHT_BACKWARD, motorSpeedRight * (-1));
  }
}
