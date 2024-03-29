#define ENCA 2
#define ENCB 3
#define PWM 5
#define IN2 6
#define IN1 7

unsigned int PPR = 0; //PPR for each motor is different

unsigned int reqAngle = 90;
float setupPPR = reqAngle*PPR/360;
volatile long currPulse = 0; 
volatile bool fired;
volatile bool up;

//to make the encoder remember its position, we could use EEPROM to embedded the current value of encoder to the MCU and when we restart, 
// we can write a code to call the store value and assigned it equals to currPulse

float Vmax = 12;
float Vmin = -12;
float V;
//PID value
float kp = 0;
float ki = 0;
float kd = 0; 

float prevError = 0;
float currError;
float prevInte = 0;
float currInte; 
unsigned int prevTime = 0;
unsigned int currTime; 

void setup() {
  Serial.begin(9600);
  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);
  // If motor turns clockwise(?), currPulse will increase.
  attachInterrupt(digitalPinToInterrupt(ENCA), isr, CHANGE);
  // //If motor turns anticlockwise(?), currPulse will decrease.
  // attachInterrupt(digitalPinToInterrupt(ENCB), antiPulses, RISING);
  
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  // Serial.println("target pos");
}

void loop() {
  currTime = millis();
  // V = pidCal(); 
  Serial.println(currPulse);
  // WriteDriverVoltage(V);
  rotateCount();
  delay(10);
}



void WriteDriverVoltage(float V) {
  int PWMval = int(255 * abs(V) / Vmax);
  if (PWMval > 255) {
    PWMval = 255;
  }
  if (V > 0) { //Not sure about Pin need to fix later
    digitalWrite(IN2, HIGH);
    digitalWrite(IN1, LOW);
  }
  else if (V < 0) { //Not sure about Pin need to fix later
    digitalWrite(IN2, LOW);
    digitalWrite(IN1, HIGH);
  }
  else {
    digitalWrite(IN2, LOW);
    digitalWrite(IN1, LOW);
  }
//  analogWrite(/*drive Pin here*/, PWMval);/
}

float pidCal(){
  // currPPR can be acquired from encoder. The lowest value is 0 and the highest value is max(PPR)
  // currPPR will increase/decrease based on the turning direction.
  currError = currPulse - setupPPR; //setupPPR can determine the angle.
  unsigned int dt = currTime - prevTime;
  currInte = prevInte + (dt * (currError + prevError))/2;

  float p = kp * currError;
  float i = ki * currInte;
  float d = kd * (currError - prevError)/dt;

  float V = p + i +d; // V is the result from PID equation
  
  
  if (V > Vmax) {
      V = Vmax;
  }
  else if (V < Vmin) {
      V = Vmin;
  }
  prevError = currError;
  prevTime = currTime; 
  prevInte = currInte;
  return V ;
}

void proPulses() {
  // Not sure that we need to set some conditions like 
  // if currPulse > /*max pulse*/{
  //   currPulse = 0;
  // }
  // else if currPulse < 0{
  //   currPulse = 360
  // }
  currPulse++;
}
void antiPulses(){
  // Not sure that we need to set some conditions like 
  // if currPulse > /*max pulse*/{
  //   currPulse = 0;
  // }
  // else if currPulse < 0{
  //   currPulse = 360
  // }
  currPulse--;
}

// Interrupt Service Routine for a change to encoder pin A
void isr ()
{
  if (digitalRead (ENCA))
    up = digitalRead (ENCB);
  else
    up = !digitalRead (ENCB);
  fired = true;
}  // end of isr

void rotateCount(){
  if (fired)
    {
    if (up)
      currPulse++;
    else
      currPulse--;
    fired = false;

    Serial.print ("Count = ");  
    Serial.println (currPulse);
    }  // end if fired

}