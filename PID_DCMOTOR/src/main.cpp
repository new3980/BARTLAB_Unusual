#include <Arduino.h>

// Motor
int motorPin1 = 4; 
int motorPin2 = 5; 
int enablePin = 6;

//Encoder
int encoderA = 2;
int encoderB = 3;

const double Kp = 3;
const double Ki = 0.001;
const double Kd = 1;

// Setting PWM properties
// const int freq = 30000; //Higher frequency = better movement flow
// const int pwmChannel = 0;
// const int resolution = 8;
// int dutyCycle = 255;

unsigned long encoderCount = 0;
unsigned long previousTime = 0;
unsigned long currentTime = 0;
double RPM = 0;
double targetRPM = 100;
double error = 0;
double timeTook = 0;
double previousError = 0;
double derivative = 0;
double cumError = 0;
double PIDcontrol = 0;
int previousCount = 0;

void Encoder()
{
  encoderCount++;
}

void setup() {
  // sets the pins as outputs:
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);

  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderA), Encoder, RISING);

  Serial.begin(115200);

  // testing
  Serial.println("Initializing...");

}

void loop() {
  currentTime = millis();
  if (currentTime - previousTime >= 50){
    RPM = (encoderCount * 20 * 60) / 337;
    error = targetRPM - RPM;

    timeTook = currentTime - previousTime;

    cumError += (error)*timeTook;

    derivative = (error - previousError) / timeTook;

    PIDcontrol = (Kp * error) + (Ki * cumError) + (Kd * derivative);
    PIDcontrol = abs(PIDcontrol);

    if (PIDcontrol > 255){
      PIDcontrol = 255;
    } else if (PIDcontrol < 0){
      PIDcontrol = 0;
    }
    
    // Serial.print("PP50ms: ");
    // Serial.println(encoderCount);
    Serial.print("RPM: ");
    Serial.print(RPM);
    Serial.print(" PID: ");
    Serial.println(PIDcontrol);

    analogWrite(enablePin, PIDcontrol);
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
    
    encoderCount = 0;
    previousError = error;
    previousTime = currentTime;

  }
  // digitalWrite(motorPin1, HIGH);
  // digitalWrite(motorPin2, LOW);
  // analogWrite(enablePin,56);

}