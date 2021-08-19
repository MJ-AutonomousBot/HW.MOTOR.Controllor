
#define AMotorPWM  10 // A모터 PIN
#define AMotorlog_a 9
#define AMotorlog_b 8

#define BMotorPWM   5 // B모터 PIN
#define BMotorlog_a 41
#define BMotorlog_b 43

boolean Forward = true;
boolean Backward = false;

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

void setup() {
  // put your setup code here, to run once:

}

void loop() {
 AMotor (100 , Backward);
 BMotor (100 , Backward);
}
