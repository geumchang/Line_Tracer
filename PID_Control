#include <IRremote.h> //IR 리모컨 라이브러리.

#define PIN_IR_RECEIVE 2
#define PIN_MOTOR_LEFT_FORWARD 10
#define PIN_MOTOR_LEFT_BACKWARD 9
#define PIN_MOTOR_RIGHT_FORWARD 6
#define PIN_MOTOR_RIGHT_BACKWARD 5
#define PIN_LINE_DETECT_RIGHT A0
#define PIN_LINE_DETECT_LEFT  A1

#define REF_VALUE_LINE_DETECT  850

#define KP  2.533
#define KI  0.0
#define KD  2.111
int motorSpeedLeft = 0; // -255~255
int motorSpeedRight = 0; // -255~255
int isStartLineTracing = 0;
int detectValueLeft = 0;
int detectValueRight = 0;

float error_prev = 0.0;  // 이전 오차값


float P_term =0.0;
float I_term = 0.0;
float D_term =0.0;

float output = 0.0;

float target = 0.0;
float error = 0.0;

int baseSpeed = 120;  // 기본 모터 속도

void processMotor(void);
void PID_control(float sensorleft, float sensorright);
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
    detectValueLeft = analogRead(PIN_LINE_DETECT_LEFT);
    detectValueRight = analogRead(PIN_LINE_DETECT_RIGHT);
    PID_control(detectValueLeft, detectValueRight);
//    if ((detectValueLeft > REF_VALUE_LINE_DETECT) && (detectValueRight > REF_VALUE_LINE_DETECT)) //센서가 둘 다 라인위에 있다
//    {
//      motorSpeedLeft = -150;
//      motorSpeedRight = -167;
//    }
//    else if ((detectValueLeft > REF_VALUE_LINE_DETECT) && (detectValueRight < REF_VALUE_LINE_DETECT)) //
//    {
//      motorSpeedLeft = -155;
//      motorSpeedRight = 70;
//    }
//    else if ((detectValueLeft < REF_VALUE_LINE_DETECT) && (detectValueRight > REF_VALUE_LINE_DETECT))
//    {
//      motorSpeedLeft = 50;
//      motorSpeedRight = -155;
//    }
//    else //왼쪽이 더빠름
//    {
//      motorSpeedLeft = 150;
//      motorSpeedRight = 167;
//    }
    }
//  processMotor();
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


void PID_control(float sensorleft, float sensorright)
{

  // PID 제어
  error = sensorright - REF_VALUE_LINE_DETECT;

  P_term = KP * error;  // 비례항 계산

  I_term += KI * error;  // 적분항 계산
  I_term = constrain(I_term, -100, 100); // 적분항 제한
  D_term = KD * (error - error_prev);  // 미분항 계산

  output = P_term + I_term + D_term;  // 출력값 계산
  // 모터 속도 계산
  motorSpeedLeft = baseSpeed - output;
  motorSpeedRight = baseSpeed + output + 17;

  // 모터 속도 제한
  motorSpeedLeft = constrain(motorSpeedLeft, -255, 255);
  motorSpeedRight = constrain(motorSpeedRight, -255, 255);
  processMotor();
  error_prev = error;  // 이전 오차값 갱신
}
