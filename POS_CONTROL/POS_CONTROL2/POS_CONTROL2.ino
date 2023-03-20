#include <util/atomic.h>

#define ENCA 2
#define ENCB 3
#define PWM 5
#define IN2 6
#define IN1 7

volatile int position = 0;
long previousTime = 0;
float previousError = 0;
float errorIntegral = 0;

void setup() {
  Serial.begin(9600);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  Serial.println("target pos");
}

void loop() {
  int target = 300 * sin(previousTime / 1e6);

  float kp = 2;
  float kd = 0.025;
  float ki = 0.1;

  long currentTime = micros();
  float deltaTime = ((float)(currentTime - previousTime)) / 1.0e6;
  previousTime = currentTime;

  int currentPosition;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    currentPosition = position;
  }

  int error = currentPosition - target;
  float errorDerivative = (error - previousError) / deltaTime;
  errorIntegral += error * deltaTime;

  float controlSignal = kp * error + kd * errorDerivative + ki * errorIntegral;

  float power = abs(controlSignal);
  if (power > 255) {
    power = 255;
  }

  setMotor(controlSignal, power, PWM, IN1, IN2);
  previousError = error;

  Serial.print(target);
  Serial.print(" ");
  Serial.print(currentPosition);
  Serial.println();
}

void setMotor(float controlSignal, int power, int pwm, int in1, int in2) {
  analogWrite(pwm, power);
  digitalWrite(in1, controlSignal > 0 ? HIGH : LOW);
  digitalWrite(in2, controlSignal > 0 ? LOW : HIGH);
}

void readEncoder() {
  int b = digitalRead(ENCB);
  position += (b > 0) ? 1 : -1;
}
