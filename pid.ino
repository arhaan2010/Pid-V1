#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

float yawrate = 0;
float calibratedyawrate = 0;
float Kp =1;
float Ki =1; 1
float Kd=1;
float error =0;
float previouserror =0;
float intgeral = 0, derivative= 0;
float pid_output = 0;

const int m1 = 6;
const int m1b = 7;
const int m2 = 8;
const int m2b = 9;
const int ena = 3;
const int enb = 5;

int i = 0;
void Pid(){
  readgyro();
  error=0-yawrate;
  intgeral+=error;
  derivative = error-previouserror;
  pid_output = Kp*error+Ki*intgeral*+Kd*derivative;
  previouserror = error;
  int basespeed =150;
  int leftmotorspeed, rightmotorspeed;
  leftmotorspeed=constrain(basespeed+pid_output,0,255);
  rightmotorspeed=constrain(basespeed+pid_output,0,255);
  analogWrite(ena,rightmotorspeed);
  analogWrite(enb,leftmotorspeed);
  Serial.println(rightmotorspeed);
   Serial.println(leftmotorspeed );
}

void setup() {

  Wire.begin();
  Serial.begin(9600);

  pinMode(m1, OUTPUT);
  pinMode(m1b, OUTPUT);
  pinMode(m2, OUTPUT);
  pinMode(m2b, OUTPUT);
  pinMode(ena, OUTPUT);
  pinMode(enb, OUTPUT);

  analogWrite(ena, 150);
  analogWrite(enb, 150);
  
  mpu.initialize();

  if(!mpu.testConnection()){
    Serial.println("Mpu not connected");
    while (1);
  }
  Serial.println("Mpu is connected... Calibrating");
  calibration();
  Serial.println("Calibrating complete");

  // for (int i = 0; i < 4; i++) 
  // {
  //   controlmotor('f');
  //   delay(2000);
  //   controlmotor('r');
  //   delay(400);
  // }
  // controlmotor('s');
 }

void controlmotor(char cmd) {
  switch(cmd){
    Pid();
    case 'l':
    digitalWrite(m1, HIGH);
    digitalWrite(m1b, LOW);
    digitalWrite(m2, HIGH);
    digitalWrite(m2b, LOW);
    break;
    case 'r':
    digitalWrite(m1, LOW);
    digitalWrite(m1b, HIGH);
    digitalWrite(m2, LOW);
    digitalWrite(m2b, HIGH);
    break;
    case 'b':
    digitalWrite(m1, HIGH);
    digitalWrite(m1b, LOW);
    digitalWrite(m2, LOW);
    digitalWrite(m2b, HIGH);
    break;
    case 'f':
    digitalWrite(m1, LOW);
    digitalWrite(m1b, HIGH);
    digitalWrite(m2, HIGH);
    digitalWrite(m2b, LOW);
    break;
    case 's':
    digitalWrite(m1, LOW);
    digitalWrite(m1b, LOW);
    digitalWrite(m2, LOW);
    digitalWrite(m2b, LOW);
    return;
    
  }
}

void calibration(){
  const int rate = 2000;
  for (int i = 0; i < rate; i++){
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    yawrate += (float)gz/ 65.5;
    delay(1);
  }
  calibratedyawrate = yawrate/rate;


}
void readgyro(){
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);
  yawrate = (float)gz/ 65.5;
  yawrate -= calibratedyawrate;
}

void loop() {
  readgyro();
  Serial.println(yawrate);
  delay(1000);

}