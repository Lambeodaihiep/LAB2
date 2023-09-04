#include <Arduino.h>
#include <Wire.h>
#include <util/atomic.h> 

#define ENC1_1 2 // right ga25
#define ENC2_1 4 
#define ENA 6
#define EN1 9
#define EN2 10
#define ENC1_2 5 // left ga25
#define ENC2_2 3
#define EN3 8
#define EN4 7
#define ENB 11

volatile int posi_right = 0; 
volatile int posi_left = 0; 

void setMotor(int dir, int L_pwmVal, int R_pwmVal){
  analogWrite(ENA,L_pwmVal);
  analogWrite(ENB,R_pwmVal);
  if(dir == -1){ // forwar
    digitalWrite(EN1,HIGH);
    digitalWrite(EN2,LOW);
    digitalWrite(EN3,HIGH);
    digitalWrite(EN4,LOW);
  }
  else if(dir == 1){ //reverse
    digitalWrite(EN1,LOW);
    digitalWrite(EN2,HIGH);
    digitalWrite(EN3,LOW);
    digitalWrite(EN4,HIGH);
  }
  else{ // stop
    digitalWrite(EN1,LOW);
    digitalWrite(EN2,LOW);
    digitalWrite(EN3,LOW);
    digitalWrite(EN4,LOW);
  }
}

void readEncoder_right(){
  int x = digitalRead(ENC1_1);
  if(x > 0)
    posi_right++;
  else
    posi_right--;
}
void readEncoder_left(){
  int x = digitalRead(ENC2_2);
  if(x > 0)
    posi_left++;
  else
    posi_left--;
}

float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch, First_angle;
uint32_t LoopTimer;
float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
float Kalman1DOutput[]={0,0};

float error, P, I, D;
float kp1 = 100.0, ki1 = 0.002, kd1 = 0.8;
float sum_error = 0, prev_error = 0, dt = 0.001;

int PWM_value, dir;

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState=KalmanState+0.004*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}

void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();     
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  RateRoll=(float)GyroX/65.5;
  RatePitch=(float)GyroY/65.5;
  RateYaw=(float)GyroZ/65.5;
  AccX=(float)AccXLSB/4096;
  AccY=(float)AccYLSB/4096;
  AccZ=(float)AccZLSB/4096;
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
}

void setup() {
  Serial.begin(115200);

  pinMode(ENC1_1,INPUT);
  pinMode(ENC1_2,INPUT);
  pinMode(ENC2_1,INPUT);
  pinMode(ENC2_2,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC1_1),readEncoder_right,RISING);
  attachInterrupt(digitalPinToInterrupt(ENC2_2),readEncoder_left,RISING);

  pinMode(ENA,OUTPUT);
  pinMode(EN1,OUTPUT);
  pinMode(EN2,OUTPUT);
  pinMode(ENB,OUTPUT);
  pinMode(EN3,OUTPUT);
  pinMode(EN4,OUTPUT);

  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  for (RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber++) {
    gyro_signals();
    RateCalibrationRoll+=RateRoll;
    RateCalibrationPitch+=RatePitch;
    RateCalibrationYaw+=RateYaw;
    delay(1);
  }
  First_angle = AngleRoll;
  //First_angle = -1.42;
  RateCalibrationRoll/=2000;
  RateCalibrationPitch/=2000;
  RateCalibrationYaw/=2000;
  LoopTimer=micros();
  Serial.print("First angle :"); Serial.println(First_angle);
  delay(2000);
}

void angle(){
  LoopTimer=micros();
  gyro_signals();
  RateRoll-=RateCalibrationRoll;
  RatePitch-=RateCalibrationPitch;
  RateYaw-=RateCalibrationYaw;
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll=Kalman1DOutput[0]; 
  KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch=Kalman1DOutput[0]; 
  KalmanUncertaintyAnglePitch=Kalman1DOutput[1];
  Serial.print("Roll Angle [°] ");
  Serial.print(KalmanAngleRoll);
  Serial.print(" Pitch Angle [°] ");
  Serial.print(KalmanAnglePitch);
  //while (micros() - LoopTimer < 4000);
  //LoopTimer=micros();
}

int PID;
void _PID(){
  error = KalmanAngleRoll - 0;
  sum_error += error;
  if (sum_error > 999999)      // nếu tổng lỗi đạt mức quá lớn thì sẽ giảm để tránh tràn số
  {
    sum_error /= 4;
  }

  P = kp1 * error;
  I = ki1 * sum_error;
  //D = kd1 * (error - prev_error) /  (micros() - LoopTimer) * 1000;
  D = kd1 * (error - prev_error) / dt;

  prev_error = error;
  PID = int(P + I + D);
  if(PID > 255 ) PID = 255;
  if(PID < -255) PID = -255;
  Serial.print(" PID value: "); Serial.print(PID);
  
}
int vl, vr;
void controlMotor(){
  Serial.print(" error: "); Serial.print(error);
  if(error > 0) dir = 1;
  else dir = -1;

  PWM_value = fabs(PID);
  if(PWM_value > 255) PWM_value = 255;
  // if(fabs(error) > 180) {
  //    vl = 0; vr = 0;
  //   }
  
  if (fabs(error) < 1.5) // nếu xe về vị trí giữa, sai số quá nhỏ thì vẫ cho xe di chuyển làm con lắc lắc về 2 phía
  {
    vl = 180;
    vr = 140;
  }
  else 
    {vl = PWM_value; vr = PWM_value - 40;}
  setMotor(dir, vl, vr);  
  Serial.print("vl = "); Serial.print(vl);
  Serial.print("vr = "); Serial.println(vr);
  delay(1);
}
 
  

void loop() {
  
  angle();
  _PID();
  controlMotor();
  delay(1);
}