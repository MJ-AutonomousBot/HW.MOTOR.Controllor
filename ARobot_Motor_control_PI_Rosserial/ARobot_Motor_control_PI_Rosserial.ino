
#define AMotorPWM  10 // A모터 PIN
#define AMotorlog_a 9
#define AMotorlog_b 8

#define BMotorPWM   5 // B모터 PIN
#define BMotorlog_a 41
#define BMotorlog_b 43

#include <ros.h>
#include <geometry_msgs/Twist.h>

// 함수 원형
void AMotor (int PWM , boolean Mode);
void BMotor (int PWM , boolean Mode);
void LDetectA(void); // 인터럽트 콜백 함수
void LDetectB(void);
void RDetectA(void);
void RDetectB(void);
int Limit(double value); // 함수정의문에서 값 조정하여 사용

// Hall 센서
int LhallA = 19;
int LhallB = 18;
int RhallA = 20;
int RhallB = 21;

// 방향을 알기 위한 Gap 값과 Micro단위에서의 작동 시간을 구하기 위한 변수 선언
volatile unsigned int LAmicro = 0, RAmicro = 0;
volatile unsigned int LBmicro = 0, RBmicro = 0;
volatile unsigned int LpreMicro = 0, RpreMicro = 0;
volatile unsigned int LbaGap = 0, LabGap = 0, RbaGap = 0, RabGap = 0;

// 회전속도 : RPM  //선속도 : m/s
unsigned int LcurRpm = 0, RcurRpm = 0;
float Lcurvel = 0, Rcurvel = 0;

//Motor 함수에서 방향을 설정하기 위한 Bool형 자료
boolean Forward = true;
boolean Backward = false;

// 모터 목표 속도 및 듀티에 관련한 변수 초기화 선언
float goal_vel = 0;
int L_duty = 0, R_duty = 0;

//PID 제어 관련 변수 선언
float LPID = 0 , LP = 0 , LI = 0, LD = 0 ;
float L_pre_error = 0, Lerorr = 0;
float RPID = 0 , RP = 0 , RI = 0, RD = 0 ;
float R_pre_error = 0, Rerorr = 0;
float Kp = 0, Kd = 0, Ki = 0 ;

ros::NodeHandle  nh;

void messageCb( const geometry_msgs::Twist& cmd_msg) {

  goal_vel = cmd_msg.linear.x;

}  // rossserial callback function

ros::Subscriber<geometry_msgs::Twist> sub_vel("cmd_vel", messageCb );

geometry_msgs::Twist cur_vel;
ros::Publisher encoder("cur_vel", &cur_vel);

void setup() {
  nh.initNode();
  nh.advertise(encoder);
  nh.subscribe(sub_vel);
  pinMode(AMotorPWM, OUTPUT);
  pinMode(AMotorlog_a, OUTPUT);
  pinMode(AMotorlog_b, OUTPUT);

  pinMode(BMotorPWM, OUTPUT);
  pinMode(BMotorlog_a, OUTPUT);
  pinMode(BMotorlog_b, OUTPUT);

  pinMode( LhallA, INPUT_PULLUP);
  pinMode( LhallB, INPUT_PULLUP);
  pinMode( RhallA, INPUT_PULLUP);
  pinMode( RhallB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LhallA), LDetectA, RISING); //DetectA Function은 속도 측정을 위한 Rising Edge에서의 인터럽트
  attachInterrupt(digitalPinToInterrupt(LhallB), LDetectB, RISING); //DetectB Function은 방향 측정을 위한 Falling Edge에서의 인터럽트
  attachInterrupt(digitalPinToInterrupt(RhallA), RDetectA, RISING);
  attachInterrupt(digitalPinToInterrupt(RhallB), RDetectB, RISING);

  Kp = 50;
  Kd = 1;
  Ki = 10;
}



void loop() {


  if ( goal_vel > 0 ) {
    Lerorr = goal_vel - Lcurvel ;
    LP = Kp * Lerorr ;
    LI = LI + Ki * Lerorr  ;
    if ( LI > 200) {
      LI = 150;
    }
    LD = Kd * (Lerorr - L_pre_error) ;
    LPID = LP + LI + LD ;
    L_pre_error = Lerorr;
    L_duty = LPID;

    Rerorr = goal_vel - Rcurvel ;
    RP = Kp * Rerorr ;
    RI = RI + Ki * Rerorr  ;
    if ( RI > 200) {
      RI = 150;
    }
    RD = Kd * (Rerorr - R_pre_error) ;
    RPID = RP + RI + RD ;
    R_pre_error = Rerorr;
    R_duty = RPID;
    AMotor(L_duty, Forward);
    BMotor(R_duty, Forward);
  }

  else if ( goal_vel < 0 ) {
    Lerorr = goal_vel - Lcurvel ;
    LP = Kp * Lerorr ;
    LI = LI + Ki * Lerorr  ;
    if ( LI > 200) {
      LI = 150;
    }
    LD = Kd * (Lerorr - L_pre_error) ;
    LPID = LP + LI + LD ;
    L_pre_error = Lerorr;
    L_duty = LPID;

    Rerorr = goal_vel - Rcurvel ;
    RP = Kp * Rerorr ;
    RI = RI + Ki * Rerorr  ;
    if ( RI > 200) {
      RI = 150;
    }
    RD = Kd * (Rerorr - R_pre_error) ;
    RPID = RP + RI + RD ;
    R_pre_error = Rerorr;
    R_duty = RPID;
    AMotor(abs(L_duty), Backward);
    BMotor(abs(L_duty), Backward);
  }
  else {
    AMotor(0, Forward);
    BMotor(0, Forward);
  }

  cur_vel.linear.x = Lcurvel;
  encoder.publish(&cur_vel);
  nh.spinOnce();
  delay(1);
}

void LDetectA()
{
  LAmicro = micros();
  LbaGap = LAmicro - LBmicro;
  if (LpreMicro) {
    LcurRpm = 1000000 * 60 / (LAmicro - LpreMicro );
    LcurRpm /= 925;
    Lcurvel = LcurRpm * (3.14159 * 0.066) / 60.0;
    //2체배| 회전당 13 PPR | 기어비(75:1) -> 13*75 =925
  }
  LpreMicro = LAmicro;
}

void LDetectB() {
  LBmicro = micros();
  LabGap = LBmicro - LAmicro;
}

void RDetectA()
{
  RAmicro = micros();
  RbaGap = RAmicro - RBmicro;
  if (RpreMicro) {
    RcurRpm = 1000000 * 60 / (RAmicro - RpreMicro );
    RcurRpm /= 925;
    Rcurvel = RcurRpm * (3.14159 * 0.066) / 60.0;
  }
  RpreMicro = RAmicro;
}

void RDetectB() {
  RBmicro = micros();
  RabGap = RBmicro - RAmicro;
}

void AMotor (int PWM , boolean Mode) // Motor Pwm Control Function
{
  if (Mode == Forward ) {
    analogWrite(AMotorPWM, PWM);
    digitalWrite(AMotorlog_a, HIGH);
    digitalWrite(AMotorlog_b, LOW);
  }
  else if (Mode == Backward)
  {
    analogWrite(AMotorPWM, PWM);
    digitalWrite(AMotorlog_a, LOW);
    digitalWrite(AMotorlog_b, HIGH);
  }


}

void BMotor (int PWM , boolean Mode) // Motor Pwm Control Function
{
  if (Mode == Forward ) {
    analogWrite(BMotorPWM, PWM);
    digitalWrite(BMotorlog_a, HIGH);
    digitalWrite(BMotorlog_b, LOW);
  }
  else if (Mode == Backward)
  {
    analogWrite(BMotorPWM, PWM);
    digitalWrite(BMotorlog_a, LOW);
    digitalWrite(BMotorlog_b, HIGH);
  }
}

int Limit(double value) {
  if (value < -100) {
    value = -100;
  }
  else if (value > 100) {
    value = 100;
  }
  return value;
}
