// Declare motor pin parameters
#define encA 2
#define encB 4
#define input1 7
#define input2 8
#define control 5

// Declare motor preperty
const unsigned long ppr = 110;
// #define ppr 110 // Depends on the motor we use 

// Declare more parameters
volatile long pulseCount = 0; 
long prevMillis = 0;
long currMillis = 0;
double currRPM;
double targRPM = 100; 
double writePulse;

double error = 0;
float prevError = 0;
float cummError = 0;

// PID 
double pid = 0;
// ki kp kd controls
float kp = 2;
float ki = 0.0001;
float kd = 2;

// Time interval for RPM calculation
unsigned long timeInterval = 50; 

// int test = map()
void setup() {
 
  Serial.begin(115200); 
  pinMode(encA , INPUT_PULLUP);
  pinMode(encB , INPUT);

  pinMode(input1, OUTPUT);
  pinMode(input2, OUTPUT);
  pinMode(control, OUTPUT);
  
  analogWrite(control, 255);
  digitalWrite(input1, HIGH);
  digitalWrite(input2, LOW);
  attachInterrupt(digitalPinToInterrupt(encA), pulses, RISING);
  
}

 
void loop() {
  currMillis = millis();
  if (currMillis - prevMillis >= timeInterval) {
    currRPM = (pulseCount *60*(1000/timeInterval))/(ppr);
    
    Serial.print(" Speed: ");
    Serial.print(currRPM);
    Serial.println(" RPM");

    error = targRPM - currRPM;
    cummError += error; 
    pid = kp*error + ki*cummError + kd*(error-prevError);
    pid = constrain((pid/850)*255, 0, 255);
    //850 is the maximum of RPM when applying supply voltage 12V to the motor.
    analogWrite(control, pid);
    digitalWrite(input1, HIGH);
    digitalWrite(input2, LOW);
    pulseCount = 0;
    prevError = error;
    prevMillis = currMillis;
  }

}


// Increment the number of pulses by 1 
void pulses() {
  pulseCount++;
}
