#include <Arduino.h>

// Declare prereqs.
const int bitResolution = 12;
int bitRange = 4096;
double timeTook, readAngle, error, errorLast;
double cumError, rateError;
double targetAngle = 305;
const int freq = 100;

const double Kp = 15;
const double Ki = 0.001;
const double Kd = 1;

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

  currentTime = millis();
  timeTook = currentTime - previousTime;

  double clockwise_distance = targetAngle - readAngle;
  if (clockwise_distance < 0) {
    clockwise_distance += 360;
  }
  double counter_clockwise_distance = readAngle - targetAngle;
  if (counter_clockwise_distance < 0) {
    counter_clockwise_distance += 360;
  }
  
  if (clockwise_distance < counter_clockwise_distance) {
    error = clockwise_distance;
  } else {
    error = -counter_clockwise_distance;
  }

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
  } else if (error == 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  previousTime = currentTime;
  errorLast = error;

  Serial.print("Angle: ");
  Serial.print(readAngle);
  Serial.print("  Error: ");
  Serial.println(error);
  delay(10);
}