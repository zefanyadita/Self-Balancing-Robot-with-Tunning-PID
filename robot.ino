//Deklarasi Librari
#include <math.h>
#include <I2Cdev.h>
#include "Wire.h"
#include <MPU6050.h>
#include <Wire.h>
#define RESTRICT_PITCH

//Deklarasi Variabel
float Kp,Ki,Ti,Td,Kd;
float Sv,Pid,Pid_1;
float interval_elapsed,interval_limit;
float et,et_1,et_2,eint,edif,eint_1,eint_update;
int Mv;
int in1 = D8;
int in2 = D7;
int in3 = D6;
int in4 = D5;
int enA = D3;
int enB = D4;

//Kalman Filter
MPU6050 accelgyro;
float Qacc, Qgyro, R;
float Xkacc, Xkpacc, Xkgyro, Pkp00, Pkp01, dXgyro;
float Pkp[2][2], KG[2], y, Wk;
unsigned long dtact;
double dtlast, dt;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t accX, accY, accZ;
double gyroY, gyroYlast, gyroYangle;
double kalmanY, PVpitch, pitch;
float Ts;
int New_PVpitch;
unsigned long t,t_1;
void setAngleY(float pitch){
  Xkacc = pitch;
  }
  
float kalman_gyro(){
  return dXgyro;
  }

float kalman_calculation(float Xtacc, float Xtgyro, float dt){
  dXgyro = Xtgyro - Xkgyro;
  Xkacc = Xkacc + (dt*dXgyro) + Wk;
  Pkp[0][0] = Pkp[0][0] + (dt*(dt*Pkp[1][1] - Pkp[0][1] - Pkp[1][0] + Qacc));
  Pkp[0][1] = Pkp[0][1] - (dt*Pkp[1][1]);
  Pkp[1][0] = Pkp[1][0] - (dt*Pkp[1][1]);
  Pkp[1][1] = Pkp[1][1] + (dt*Qgyro);
  KG[0] = Pkp[0][0]/(Pkp[0][0] + R);
  KG[1] = Pkp[1][0]/(Pkp[0][0] + R);
  Xkacc = Xkacc + (KG[0]*(Xtacc - Xkacc));
  Xkgyro = Xkgyro + (KG[1]*(Xtacc - Xkacc));
  Pkp00 = Pkp[0][0];
  Pkp01 = Pkp[0][1];
  Pkp[0][0] = Pkp[0][0] - (KG[0] * Pkp00);
  Pkp[0][1] = Pkp[0][1] - (KG[0] * Pkp01);
  Pkp[1][0] = Pkp[1][0] - (KG[1] * Pkp00);
  Pkp[1][1] = Pkp[1][1] - (KG[1] * Pkp01);
  return Xkacc;
}
void requestEvent(){
Wire.write(New_PVpitch);
}

//Driver Motor
void plus(){
  analogWrite(enA,Mv*-1);
  analogWrite(enB,Mv*-1);
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);
}
void minus(){
  analogWrite(enA,Mv);
  analogWrite(enB,Mv);
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  digitalWrite(in3,LOW);
  digitalWrite(in4,HIGH);
}
void nol(){
  analogWrite(enA,0);
  analogWrite(enB,0);
  digitalWrite(in1,LOW);
  digitalWrite(in2,LOW);
  digitalWrite(in3,LOW);
  digitalWrite(in4,LOW);
}
void Motor_Control(){
  if (Mv < 10){
  plus();
  }
  else if (Mv > -10){
  minus();
  }
  else{
  nol();
  }
}

void setup() {
//Perhitungan Kalman Filter
dt = 0;
dtlast = 0;
Qacc = 0.0002;
Qgyro = 0.0004;
R = 0.2;
Xkacc = 0.0;
Xkpacc = 0.0;
Xkgyro = 0.0;
Pkp[0][0] = 0.0;
Pkp[0][1] = 0.0;
Pkp[1][0] = 0.0;
Pkp[1][1] = 0.0;
Ts = 0.1;
Wire.begin();
Serial.begin(9600);
accelgyro.initialize();

//Setup Nilai Kendali
Kp = 162.5;
Ti = 0.000105;
Td = 66; 
if (Ti<=0){
 Ki=0;
  }
else {
 Ki=Kp/Ti;
}
Kd=Kp*Td;
et_1 = 0;
eint_1=0;
interval_limit = 1;
interval_elapsed = 0;

//Setup Pin yang digunakan
pinMode(in1,OUTPUT);
pinMode(in2,OUTPUT);
pinMode(in3,OUTPUT);
pinMode(in4,OUTPUT);
pinMode(enB,OUTPUT);
pinMode(enA,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
//Kalman Filter
  Sv = 0;
  dtlast = dtact;
  dtact = millis();
  dt = dtact;
  dt = (dt - dtlast)/1000;
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  accX = ax; accZ = az;
  pitch = atan2(-accX, accZ)*RAD_TO_DEG;
  gyroY = gy;
  gyroY = gyroY / 131.0;
  
  if ((pitch < -90 && kalmanY > 90) || (pitch > 90 && kalmanY < -90)) {
  setAngleY(pitch);
  kalmanY = pitch;
  gyroYangle = pitch;
  }
  else{
    kalmanY = kalman_calculation(pitch, gyroY, dt);
    }
    
  gyroYangle = gyroYangle + (kalman_gyro()*dt);

if (gyroYangle < -180 || gyroYangle > 180){
  gyroYangle = kalmanY;
  }
  
  PVpitch = kalmanY;
  PVpitch = PVpitch - 0;
  New_PVpitch = PVpitch;
  //batas kemirigan
      if (New_PVpitch > 25 || New_PVpitch < -25){
    New_PVpitch = 0;
  }
  
 //t=millis();
 //Ts = (t - t_1)/1000;
 Ts = 0.1;
  //hitung error
  et=Sv-New_PVpitch;
  eint_update = ((et + et_1)*Ts)/2;
  eint = eint_1 + eint_update;
  
  // hitung differential error
  edif = (et - et_1)/Ts;
  //hitung Pid
  Pid = Kp*et + Ki*eint + Kd*edif;
  if (Pid > 120) 
  {
    Pid = 120;
  }
  else if (Pid < -120){
    Pid=-120;
  }
  else {
    Pid = Pid;
  }
  Mv = Pid;
Motor_Control();

  //hitung waktu display
   interval_elapsed=interval_elapsed + Ts;
   if(interval_elapsed >= interval_limit){
    Serial.print(New_PVpitch);
    Serial.print(" ");
    Serial.print(0);
    Serial.print(" ");
    Serial.print(25);
    Serial.print(" ");
    Serial.print(-25);
    Serial.print(" ");
    Serial.println(Sv);
        interval_elapsed=0;
   }
      else{
    interval_elapsed=interval_elapsed;
   }
}