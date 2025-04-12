#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

float kp = 1.5, ki = 0.1, kd = 0.1;
float error = 0, previouserror = 0;
float integral = 0, derivative = 0;
float pidoutput;
float yawangle =0;
unsigned long previous_time =0;




//Variable declare
float yawrate = 0;
float calibratedyawrate = 0;

const int m1a = 6;
const int m1b = 7;
const int m2a = 8;
const int m2b = 9;
const int ena = 3;
const int enb = 5;

//Variable declare
void turn(float target_angle){
  yawangle=0;



}
void PID() {
  readgyro();
  error = 0 - yawrate;
  integral += error;
  derivative = error - previouserror;

  pidoutput = kp* error + ki* integral + kd* derivative;
  previouserror = error;

  int basespeed = 200;
  int leftmotorspeed, rightmotorspeed;

  leftmotorspeed = constrain(basespeed + pidoutput, 0, 255);
  rightmotorspeed = constrain(basespeed - pidoutput, 0, 255);

  analogWrite(ena, rightmotorspeed);
  analogWrite(enb, leftmotorspeed);

  Serial.print("rightmotorspeed  ");
  Serial.print(rightmotorspeed);

  Serial.print("   leftmotorspeed   ");
  Serial.println(leftmotorspeed);
}

void setup() {

  Wire.begin();
  Serial.begin(9600);

  pinMode(m1a, OUTPUT);
  pinMode(m1b, OUTPUT);
  pinMode(m2a, OUTPUT);
  pinMode(m2b, OUTPUT);
  pinMode(ena, OUTPUT);
  pinMode(enb, OUTPUT);
  analogWrite(ena, 150);
  analogWrite(enb, 150);


  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU not Connected ");
    while (1)
      ;
  }
  Serial.println("MPU Connected, Calibrating the sensor");

  for (int i = 0; i < 2000; i++) {
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    yawrate += (float)gz / 65.5;
    delay(1);
    //Serial.print(". ");
  }
  calibratedyawrate = yawrate / 2000;

  Serial.println("Calibration Complete");

  controlmotor('f', 10000);
}

void readgyro() {
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);
  yawrate = (float)gz / 65.5;  //Convert into deg/s
  yawrate -= calibratedyawrate;
}

void controlmotor(char cmd, long duration) {

  unsigned long starttime = millis();
  unsigned long currenttime;

  while ((currenttime = millis()) - starttime < duration){ 
    PID();
    switch (cmd) {
      case 'l':
        Serial.println("in F");
        digitalWrite(m1a, HIGH);
        digitalWrite(m1b, LOW);
        digitalWrite(m2a, HIGH);
        digitalWrite(m2b, LOW);
        break;

      case 'f':
        digitalWrite(m1a, LOW);
        digitalWrite(m1b, HIGH);
        digitalWrite(m2a, LOW);
        digitalWrite(m2b, HIGH);
        break;

      case 'r':
        digitalWrite(m1a, HIGH);
        digitalWrite(m1b, LOW);
        digitalWrite(m2a, LOW);
        digitalWrite(m2b, HIGH);
        break;

      case 'b':
        digitalWrite(m1a, LOW);
        digitalWrite(m1b, HIGH);
        digitalWrite(m2a, HIGH);
        digitalWrite(m2b, LOW);
        break;
      case 's':
        digitalWrite(m1a, LOW);
        digitalWrite(m1b, LOW);
        digitalWrite(m2a, LOW);
        digitalWrite(m2b, LOW);
        return;
    }
  }
}
void loop() {
  // readgyro();
  // Serial.println(yawrate);
  // delay(200);
}