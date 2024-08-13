#include <stdio.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <PS2X_lib.h>

#define PS2_DAT 12
#define PS2_CMD 13
#define PS2_SEL 15
#define PS2_CLK 14

#define MIN_PWM 0
#define MAX_PWM 4095
#define MIN_SERVO_180_SPEED 204
#define MAX_SERVO_180_SPEED 480
#define MIN_SERVO_360_SPEED 93
#define MAX_SERVO_360_SPEED 440

#define Servo_180_1 7
#define Servo_180_2 3
#define Servo_360_1 4
#define Servo_360_2 5
#define Servo_360_3 6

#define PWM_CHANNEL1 8
#define PWM_CHANNEL2 9
#define PWM_CHANNEL3 10
#define PWM_CHANNEL4 11
#define PWM_CHANNEL5 12
#define PWM_CHANNEL6 13
#define PWM_CHANNEL7 14
#define PWM_CHANNEL8 15


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
PS2X ps2x;

void setup()
{
  Serial.begin(115200);
  initMotors();
  setupPS2controller();
  Serial.println("Done setup!");
}

void loop()
{
  ps2x.read_gamepad(false, 0);
  DCcontrol();
  thuBongnNhaBong();
  banBong();
  cuonThanhMoCua();
}

void setPWMMotors(int c1, int c2, int c3, int c4)
{
  pwm.setPWM(PWM_CHANNEL1, c1, MAX_PWM - c1);
  pwm.setPWM(PWM_CHANNEL2, c2, MAX_PWM - c2);
  pwm.setPWM(PWM_CHANNEL3, c3, MAX_PWM - c3);
  pwm.setPWM(PWM_CHANNEL4, c4, MAX_PWM - c4);
}

void initMotors()
{
  Wire.begin();
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  Wire.setClock(400000);
  setPWMMotors(0, 0, 0, 0);
}

void setupPS2controller()
{
  ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, true, true);
  ps2x.readType();
}


bool DCcontrol()
{
  int bien = 1600;
  float bienn = 1600.0;
  //gán giá trị cho trục x, y của joystick trái:
  int nJoyX = ps2x.Analog(PSS_LX);
  int nJoyY = ps2x.Analog(PSS_LY);
  //gán giá trị cho trục x, y của joystick phải:
  int nJoyRX = ps2x.Analog(PSS_RX);
  int nJoyRY = ps2x.Analog(PSS_RY);
  //gán giá trị cho nút tam giác:
  

  //đổi từ giá trị mặc định sang giá trị max cho joystick trái:
  nJoyX = map(nJoyX, 0, 255, -bien, bien);
  nJoyY = map(nJoyY, 0, 255, bien, -bien);

  //đổi từ giá trị mặc định sang giá trị max cho joystick phải:
  nJoyRX = map(nJoyRX, 0, 255, -bien, bien);
  nJoyRY = map(nJoyRY, 0, 255, bien, -bien);

  //Động cơ
  int nMotMixL;
  int nMotMixR;
  
  float fPivYLimit = bienn; //giới hạn độ băm xung

  float nMotPremixL;
  float nMotPremixR;
  int nPivSpeed;
  float fPivScale;
     
  if (nJoyY >= 0)
  {
    // Tiến về phía trước:
    nMotPremixL = (nJoyX >= 0) ? bienn : (bienn + nJoyX);  //nếu đi sang phải, tốc độ bánh trái lớn hơn
    nMotPremixR = (nJoyX >= 0) ? (bienn - nJoyX) : bienn;
  }
  else
  {
    // Reverse
    nMotPremixL = (nJoyX >= 0) ? (bienn - nJoyX) : bienn;
    nMotPremixR = (nJoyX >= 0) ? bienn : (bienn + nJoyX);
  }

  nMotPremixL = nMotPremixL * nJoyY / bienn;
  nMotPremixR = nMotPremixR * nJoyY / bienn;

  nPivSpeed = nJoyX;
  fPivScale = (abs(nJoyY) > fPivYLimit) ? 0.0 : (1.0 - abs(nJoyY) / fPivYLimit);

  nMotMixL = (1.0 - fPivScale) * nMotPremixL + fPivScale * (nPivSpeed);
  nMotMixR = (1.0 - fPivScale) * nMotPremixR + fPivScale * (-nPivSpeed);
  
  int c1 = 0, c2 = 0, c3 = 0, c4 = 0;
 
  if (nMotMixR > 50)
  {
    c3 = nMotMixR;
  }

  else if (nMotMixR < -50)
  {
    c4 = abs(nMotMixR);
  }

  if (nMotMixL > 50)
  {
     c1 = nMotMixL;
  }
  else if (nMotMixL < -50)
  {
     c2 = abs(nMotMixL);
  }

  setPWMMotors(c1, c2, c3, c4);
  
  delay(50);
  return 1;
}

void thuBongnNhaBong()
{
  if (ps2x.Button(PSB_TRIANGLE)){
    pwm.setPWM(PWM_CHANNEL5, 0, 1000);
    pwm.setPWM(PWM_CHANNEL6, 0, 0);
  }
  if (ps2x.Button(PSB_CROSS)){
    pwm.setPWM(PWM_CHANNEL5, 0, 0);
    pwm.setPWM(PWM_CHANNEL6, 0, 2000);
  }
  if (ps2x.Button(PSB_CIRCLE)){
    pwm.setPWM(PWM_CHANNEL5, 0, 0);
    pwm.setPWM(PWM_CHANNEL6, 0, 0);
  }
}

void cuonThanhMoCua()
{
  if (ps2x.Button(PSB_L1)){
    pwm.setPWM(Servo_360_1, 0, MIN_SERVO_360_SPEED);
    pwm.setPWM(Servo_360_2, 0, MAX_SERVO_360_SPEED);
  }
  if (ps2x.ButtonReleased(PSB_L1)){
    pwm.setPWM(Servo_360_1, 0, 0);
    pwm.setPWM(Servo_360_2, 0, 0);
  }
  if (ps2x.Button(PSB_L2)){
    pwm.setPWM(Servo_360_1, 0, MAX_SERVO_360_SPEED);
    pwm.setPWM(Servo_360_2, 0, MIN_SERVO_360_SPEED);
  }
  if (ps2x.ButtonReleased(PSB_L2)){
    pwm.setPWM(Servo_360_1, 0, 0);
    pwm.setPWM(Servo_360_2, 0, 0);
  }
  if (ps2x.Button(PSB_PAD_UP)){
    Serial.println("Nut Up duoc an");
    pwm.setPWM(Servo_360_3, 0, 410);
    delay(200);
    pwm.setPWM(Servo_360_3, 0, 0);
  }
  if (ps2x.Button(PSB_PAD_DOWN)){
    Serial.println("Nut Up khong an");
    pwm.setPWM(Servo_360_3, 0, 205);
    delay(200);
    pwm.setPWM(Servo_360_3, 0, 0);
   
  }
}

void banBong()
{
  if (ps2x.Button(PSB_R1)){
    pwm.setPWM(Servo_180_1, 0, MAX_SERVO_180_SPEED);
  }
  if (ps2x.Button(PSB_R2)){
    pwm.setPWM(Servo_180_1, 0, MIN_SERVO_180_SPEED);
  }
  
  if (ps2x.Button(PSB_SQUARE)){
    pwm.setPWM(PWM_CHANNEL7, 0, 4095);
    pwm.setPWM(PWM_CHANNEL8, 0, 0);
  }
  if (ps2x.ButtonReleased(PSB_SQUARE)){
    pwm.setPWM(PWM_CHANNEL7, 0, 0);
    pwm.setPWM(PWM_CHANNEL8, 0, 0);
  }
}
