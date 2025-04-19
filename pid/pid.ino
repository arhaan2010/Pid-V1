#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

float kp = 3.4, ki = 0.5, kd = 0.5;
float error = 0, previouserror = 0;
float integral = 0, derivative = 0;
float pidoutput;
float yawangle = 0;
unsigned long previous_time = 0;

const int wheeldia = 67;
const float wheelc = 3.141592653589793 * wheeldia;
int estimaterpm = 168;
float yawrate = 0;
float calibratedyawrate = 0;

const int m1a = 6;
const int m1b = 7;
const int m2a = 8;
const int m2b = 9;
const int ena = 3;
const int enb = 5;

void PID() {
  readgyro();
  error = 0 - yawrate;
  integral += error;
  derivative = error - previouserror;

  pidoutput = kp * error + ki * integral + kd * derivative;
  previouserror = error;

  int basespeed = 200;
  int leftmotorspeed = constrain(basespeed + pidoutput, 0, 255);
  int rightmotorspeed = constrain(basespeed - pidoutput, 0, 255);

  analogWrite(ena, rightmotorspeed);
  analogWrite(enb, leftmotorspeed);

  Serial.print("rightmotorspeed  ");
  Serial.print(rightmotorspeed);
  Serial.print("   leftmotorspeed   ");
  Serial.println(leftmotorspeed);
}

void controlmotor(char cmd, long duration) {
  unsigned long starttime = millis();
  unsigned long currenttime;

  while ((currenttime = millis()) - starttime < duration) {
    PID();
    switch (cmd) {
      case 'f':
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
      case 'r':
        digitalWrite(m1a, HIGH);
        digitalWrite(m1b, LOW);
        digitalWrite(m2a, HIGH);
        digitalWrite(m2b, LOW);
        break;
      case 'l':
        digitalWrite(m1a, LOW);
        digitalWrite(m1b, HIGH);
        digitalWrite(m2a, LOW);
        digitalWrite(m2b, HIGH);
        break;
      case 's':
        digitalWrite(m1a, LOW);
        digitalWrite(m1b, LOW);
        digitalWrite(m2a, LOW);
        digitalWrite(m2b, LOW);
        return;
    }
  }

  // Stop motors after motion
  digitalWrite(m1a, LOW);
  digitalWrite(m1b, LOW);
  digitalWrite(m2a, LOW);
  digitalWrite(m2b, LOW);
}

void cal(float distance) {
  float distancemm = distance * 10;
  float numofrev = distancemm / wheelc;
  float timeperrev = 60.0 / estimaterpm;
  float totaltimesec = numofrev * timeperrev;
  unsigned long totaltimems = totaltimesec * 1000;
  controlmotor('f', totaltimems);
}

void turn(float target_angle) {
  int turnspeed = 100;
  analogWrite(ena, turnspeed);
  analogWrite(enb, turnspeed);

  yawangle = 0;
  unsigned long currenttime = millis();
  previous_time = currenttime;

  while (abs(target_angle - yawangle) > 1.0) {
    currenttime = millis();
    float deltatime = (currenttime - previous_time) / 1000.0;
    previous_time = currenttime;
    readgyro();
    yawangle += yawrate * deltatime;



    if (target_angle > yawangle) {
      digitalWrite(m1a, HIGH);
      digitalWrite(m1b, LOW);
      digitalWrite(m2a, HIGH);
      digitalWrite(m2b, LOW);
    } else {
      digitalWrite(m1a, HIGH);
      digitalWrite(m1b, LOW);
      digitalWrite(m2a, LOW);
      digitalWrite(m2b, HIGH);
    }
    if (yawangle == target_angle / 2) {
      turnspeed = 50;
    }else if (yawangle == target_angle / 3) {
      turnspeed = 30;
    
    } else if (yawangle == target_angle / 4) {
      turnspeed = 20;
    
    } else if (yawangle == target_angle / 5) {
      turnspeed = 15;
    }

  }

  // Stop motors after turning
  digitalWrite(m1a, LOW);
  digitalWrite(m1b, LOW);
  digitalWrite(m2a, LOW);
  digitalWrite(m2b, LOW);
}



void readgyro() {
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);
  yawrate = (float)gz / 65.5;
  yawrate -= calibratedyawrate;
}



// your original untouched square function
void square() {
  for (int i = 0; i < 4; i++) {
    cal(107);
    delay(500);
    turn(69);
    delay(500);
  }
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
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);

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
  }

  calibratedyawrate = yawrate / 2000;
  Serial.println("Calibration Complete");

  square();
}

void loop() {
  // nothing here yet
}
