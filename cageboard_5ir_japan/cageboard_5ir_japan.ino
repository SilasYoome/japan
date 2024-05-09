#include "QTRSensors.h"

#define Kp 5 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 0.05 //6.8// experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define MaxSpeed 150// max speed of the robot
#define BaseSpeed 80 // this is the speed at which the motors should spin when the robot is perfectly on the line

#define speedturn 100

#define rightMotor 4
#define rightMotorPWM 6
#define leftMotor 7
#define leftMotorPWM 5


QTRSensors qtr;

const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];


void move(int motor, int speed, int direction);
void wait();


void setup()
{
  Serial.begin(9600);
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]) { A1, A2, A3, A4, A5 }, SensorCount);

  pinMode(12, INPUT);
  pinMode(7, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(6, OUTPUT);
  while ((digitalRead(12) == HIGH)) {
  }
  Serial.println("calibrate start");

  for (int j = 0;j < 1;j++) {
    for (int i = 0; i < 100;i++) {
      if (i < 25 || i >= 75) {
        move(1, 50, 1);
        move(0, 50, 0);
      }
      else
      {
        move(1, 50, 0);
        move(0, 50, 1);
      }
      qtr.calibrate();
      delay(20);
    }
  }
  wait();
  Serial.println("calibrate end");
  delay(3000);
  while ((digitalRead(12) == HIGH)) {
  }
}

int lastError = 0;
int position;

void loop()
{

  position = qtr.readLineBlack(sensorValues);

  if ((sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0) ||
    (sensorValues[0] > 0 && sensorValues[1] > 0 && sensorValues[2] > 0 && sensorValues[3] > 0 && sensorValues[4] > 0)) {
    move(1, BaseSpeed, 1);//motor derecho hacia adelante
    move(0, BaseSpeed, 1);//motor izquierdo hacia adelante
    return;
  }
  else if (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] > 200 && sensorValues[3] > 200 && sensorValues[4] > 200) {
    move(1, speedturn, 1);//motor derecho hacia adelante
    move(0, speedturn, 0);//motor izquierdo hacia adelanteZ
    delay(50);
    return;
  }
  else if (sensorValues[0] > 200 && sensorValues[1] > 200 && sensorValues[2] > 200 && sensorValues[3] == 0 && sensorValues[4] == 0) {
    move(1, speedturn, 0);//motor derecho hacia adelante
    move(0, speedturn, 1);//motor izquierdo hacia adelante
    delay(50);
    return;
  }

  int error = position - 2000;
  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;

  int rightMotorSpeed = BaseSpeed + motorSpeed;
  int leftMotorSpeed = BaseSpeed - motorSpeed;

  if (rightMotorSpeed > MaxSpeed) rightMotorSpeed = MaxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > MaxSpeed) leftMotorSpeed = MaxSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0)rightMotorSpeed = 0;
  if (leftMotorSpeed < 0)leftMotorSpeed = 0;

  /*for(int i = 0;i<SensorCount;i++){
    Serial.println(sensorValues[i]);
  }
    Serial.println();
    Serial.print("position: ");
    Serial.println(position);
    Serial.print("error: ");
    Serial.println(error);
    Serial.print("motorspeed: ");
    Serial.println(motorSpeed);
    Serial.print("rightMotorSpeed :");*/
    /*Serial.print(rightMotorSpeed);
    Serial.print(" ");
    Serial.println(leftMotorSpeed);*/
    //Serial.println();
    //delay(500);

  
  move(1, rightMotorSpeed, 1);//motor derecho hacia adelante
  move(0, leftMotorSpeed, 1);//motor izquierdo hacia adelante

}



void move(int motor, int speed, int direction) {

  boolean inPin = HIGH;
  if (direction == 1) {
    inPin = HIGH;
  }
  else if (direction == 0) {
    inPin = LOW;
  }

  if (motor == 0) {
    digitalWrite(leftMotor, !inPin);
    analogWrite(leftMotorPWM, speed);
  }
  else if (motor == 1) {
    digitalWrite(rightMotor, inPin);
    analogWrite(rightMotorPWM, speed);
  }
}

void wait() {
  analogWrite(leftMotorPWM, 0);
  analogWrite(rightMotorPWM, 0);
}
