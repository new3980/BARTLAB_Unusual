#include <Arduino.h>

// Declare prereqs.
const int bitResolution = 12;
int bitRange = 4096;
double timeTook, readAngle, error, errorLast;
double cumError, rateError;
double targetAngle = 90;
const int freq = 100;

const double Kp = 25;
const double Ki = 0.0001;
const double Kd = 10;

const int CHANNEL1 = 0;
const int angleReadPin = 34;

// L298N Motor A pins
const int enA = 18;
const int in1 = 19;
const int in2 = 21;

double Pterm, Iterm, Dterm, PID;
double currentTime, previousTime;

void setup() {

  ledcSetup(CHANNEL1, freq, bitResolution);
  ledcAttachPin(enA, CHANNEL1);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  Serial.begin(9600);

}

void loop() {
  int rawAngle = analogRead(angleReadPin);
  readAngle = (360 * rawAngle) / (bitRange - 1);
  if (readAngle==360){readAngle=0;}
  currentTime = millis();
  timeTook = currentTime - previousTime;

  error = (targetAngle - readAngle);
  cumError += (error) * timeTook;
  rateError = (error - errorLast) / timeTook;

  PID = (Kp * error) + (Ki * cumError) + Kd * rateError;
  PID = abs(PID);

  if (PID > 4096) {
    PID = 4095;
  }

  if (error > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    ledcWrite(CHANNEL1, PID);
  } else if (error < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    ledcWrite(CHANNEL1, PID);
  } else if (error==0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  } 

  previousTime = currentTime;
  errorLast = error;

  //Serial.print("Angle: ");
  Serial.println(readAngle);
  // Serial.print("Error: ");
  //Serial.println(targetAngle);
  delay(10);

}
