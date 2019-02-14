/*
* Carnegie Mellon University Mechatronics 2019: ShipBot Team A
* Sensor Lab
* Authors: David Bang (@dbang)
* Fiona Li (@siling1)
* Sara Misra (@saram1)
* Haowen Shi (@haowensh)
* Bo Tian (@btian1)
*/

#include <Servo.h>
#include <Stepper.h>
#include <Encoder.h>

Servo servoMotor;  // create servo object to control a servo
const int stepsPerRevolution = 400; //according to specifications of the stepper motor 
Stepper stepperMotor(stepsPerRevolution, 10,11,12,13); //initialize stepper library on the respective pins 

// Digital Pin Assignments
const int trigOutput = 24;
const int echoInput = 22;
const int servoMotorOutput = 8;
const int dcMotorIn1 = 7;
const int dcMotorIn2 = 6;
const int dcMotorEn = 5;
const int dcEncodeA = 2;
const int dcEncodeB = 3;
int iterationCount = 0;


Encoder myEnc(dcEncodeA,dcEncodeB);

// Analog Pin Assignments
const int sharpIRInput = 0;
const int potentiometerOutput = 7;
const int sw1Input = A4;
const int sw2Input = A5;

const int windowSize = 20;
float currentStepperAngle = 0; 

float USS_result = 0;
char last;

String command = "";
boolean sensorMode = true;

int sw1 = 1;
int sw2 = 1;

// PID
#define LOOPTIME        100                     // PID loop time
#define NUMREADINGS     10                      // samples for Amp average

int readings[NUMREADINGS];
unsigned long lastMilli = 0;                    // loop timing 
unsigned long lastMilliPrint = 0;               // loop timing
int speed_req = 300;                            // speed (Set Point)
int speed_act = 0;                              // speed (actual value)
int PWM_val = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
int voltage = 0;                                // in mV
int current = 0;                                // in mA
volatile long count = 0;                        // rev counter
float Kp =   .9;                                // PID proportional control Gain
float Kd =   .5;                                // PID Derivitave control gain

/* PID CODE */
void getMotorData()  {                                                        // calculate speed, volts and Amps
static long countAnt = 0;                                                   // last count
 int cnt_count = myEnc.read();
 speed_act = ((cnt_count - countAnt)*(60*(1000/LOOPTIME)))/(16*29);          // 16 pulses X 29 gear ratio = 464 counts per output shaft rev
 countAnt = cnt_count;                  
}

int updatePid(int command, int targetValue, int currentValue)   {             // compute PWM value
float pidTerm = 0;                                                            // PID correction
int error=0;                                  
static int last_error=0;                             
 error = abs(targetValue) - abs(currentValue); 
 pidTerm = (Kp * error) + (Kd * (error - last_error));                            
 last_error = error;
 return constrain(command + int(pidTerm), 0, 255);
}



void setup() {

  //Pin Setup
  pinMode(trigOutput, OUTPUT);
  pinMode(echoInput, INPUT);
  pinMode(sharpIRInput, INPUT);
  pinMode(potentiometerOutput, OUTPUT);
  Serial.begin(9600);
  servoMotor.attach(servoMotorOutput);

  stepperMotor.setSpeed(45);
  
  //Switch Pin Setup
  pinMode(sw1Input, INPUT);
  pinMode(sw2Input, INPUT);

  pinMode(dcMotorEn, OUTPUT);

  for(int i=0; i<NUMREADINGS; i++)   readings[i] = 0;  // initialize readings to 0

}

void outputAllValuesForUI(){
  int restriction = 300;
  if (!sensorMode){
    restriction = 1800;
  }
  if (iterationCount >= restriction + 30) {
    iterationCount = 0;
    onlyReadandPrintSensorValues();
    onlyReadandPrintMotorValues();
    iterationCount++;
    Serial.println("Go111");
  }
  if (iterationCount == (restriction/3 * 2)){
    Serial.println("NoGo");
  }
  iterationCount++;
}

void guiMode() { 
  while (Serial.available() > 0) {
    int inChar = Serial.read();

    if (inChar != '\n') { 
      command += (char)inChar;
    }
    else {
      //Serial.println("This is Commands ");
      //Serial.println(command);
      if (command.indexOf("SenC:") != -1) {
        String value = command.substring(command.lastIndexOf(':') + 1);
        if (value.equals("false")){
          sensorMode = false;
        }
        else if (value.equals("true")){
          sensorMode = true;
        }
      }
      else if (sensorMode == false) {
        if (command.indexOf("SC:") != -1) {
          String value = command.substring(command.lastIndexOf(':') + 1);
          updateStepper(value.toFloat());
        }
        else if (command.indexOf("SerC:") != -1) {
          String value = command.substring(command.lastIndexOf(':') + 1);
          updateServo(value.toFloat());
        }
        else if (command.indexOf("DCAC:") != -1) {
          String value = command.substring(command.lastIndexOf(':') + 1);
          updateDCAngle(value.toFloat());
        }
        else if (command.indexOf("DCVC:") != -1) {
          String value = command.substring(command.lastIndexOf(':') + 1);
          updateDCVel(value.toFloat());
        }
      }
      command = "";
    }
  }
  
}

void updateStepper(float angle){
  float angleStepper = angle-currentStepperAngle; 
  int steps = int((angleStepper/0.9))  % stepsPerRevolution;
  currentStepperAngle = angle;
  
  stepperMotor.step(steps);
}

void updateServo(float angle){
  servoMotor.write(angle);
}

void updateDCAngle(float angle){

}

void updateDCVel(float vel){
  
  speed_req = constrain(map(int(vel), 0, 20, 60, 255), 0, 255);

  digitalWrite(dcMotorIn1, HIGH);
  digitalWrite(dcMotorIn2, LOW);
  
  
  if((millis()-lastMilli) >= LOOPTIME)   {                                    // enter tmed loop
     lastMilli = millis();
     getMotorData(); // calculate speed, volts and Amps
     PWM_val = updatePid(PWM_val, speed_req, speed_act); // compute PWM value
     analogWrite(dcMotorEn, PWM_val); // send PWM to motor
  }
}

void loop() {
  outputAllValuesForUI();
  guiMode();
  
  if (sensorMode){
    readSwitch(&sw1, &sw2);
    if ((sw1 == 0) && (sw2 == 0)) {
      readUltraSoundSensor();
      for(int i=0; i<NUMREADINGS; i++)   readings[i] = 0; 
    } 
    else if ((sw1 == 0) && (sw2 == 1)) {
      readIRSensor();
    }
    else if ((sw1 == 1) && (sw2 == 0)) {
      readPotentiometerSensor();
      for(int i=0; i<NUMREADINGS; i++)   readings[i] = 0; 
    }
    else {
      for(int i=0; i<NUMREADINGS; i++)   readings[i] = 0;   
    }
  }
}

void insertionSort(float* arr, int n){ 
 int i, j; 
 float key;
 for (i = 1; i < n; i++){ 
     key = arr[i]; 
     j = i-1; 
     while (j >= 0 && arr[j] > key) 
     { 
         arr[j+1] = arr[j]; 
         j = j-1; 
     } 
     arr[j+1] = key; 
 } 
} 

float findMedian(float* median, int size){
  insertionSort(median, windowSize);
  return median[windowSize/2 - 1];
}

void onlyReadandPrintSensorValues(){
  //ultraSound
  float medianFilter[windowSize];
  // Sets the trigPin on HIGH state for 10 micro seconds
  for (int i = 0; i < windowSize; i++){
    digitalWrite(trigOutput, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigOutput, LOW);
    
    // Reads the echoPin, returns the sound wave travel time in microseconds
    float duration = pulseIn(echoInput, HIGH);
    // Calculating the distance
    float distance= duration*0.034/2;
    // Prints the distance on the Serial Monitor
    if (duration > 30000) {
      // invalid reading
      duration = USS_result;
    } else {
      USS_result = duration;
    }
    float cdis = 0.0067*duration+ 0.6603;
    medianFilter[i] = cdis;
  }
  float ultraSound = findMedian(medianFilter, windowSize);

  //IR
  for (int i = 0; i < windowSize; i++){
    float volts = analogRead(sharpIRInput) *(5.0 / 1024.0);
    float Dis = 4.4627*volts*volts-19.378*volts+24.957;
    medianFilter[i] = Dis;
  }
  float ir = findMedian(medianFilter, windowSize);

  //Potentiometer
  for (int i = 0; i < windowSize; i++){
    float volts = analogRead(potentiometerOutput) *(5.0 / 1024.0);
    float resis = (2.9263*volts*volts-22.403*volts+97.623)*volts;
    medianFilter[i] = resis;
  }
  float potentiometer = findMedian(medianFilter, windowSize);

  Serial.print("UltraSound:");
  Serial.println(ultraSound);
  Serial.print("IR:");
  Serial.println(ir);
  Serial.print("Potentiometer:");
  Serial.println(potentiometer);  
}

void onlyReadandPrintMotorValues(){
  int servoAngle = servoMotor.read();
  int stepperAngle = currentStepperAngle;
  int dcAngle = myEnc.read() * 2;

  Serial.print("ServoAngle:");
  Serial.println(servoAngle);
  Serial.print("StepperAngle:");
  Serial.println(stepperAngle);
  Serial.print("DCAngle:");
  Serial.println(dcAngle);
}

float readUltraSoundSensor(){
  float medianFilter[windowSize];
  // Sets the trigPin on HIGH state for 10 micro seconds
  for (int i = 0; i < windowSize; i++){
    digitalWrite(trigOutput, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigOutput, LOW);
    
    // Reads the echoPin, returns the sound wave travel time in microseconds
    float duration = pulseIn(echoInput, HIGH);
    // Calculating the distance
    float distance= duration*0.034/2;
    // Prints the distance on the Serial Monitor
    if (duration > 30000) {
      // invalid reading
      duration = USS_result;
    } else {
      USS_result = duration;
    }
    float cdis = 0.0067*duration+ 0.6603;
    medianFilter[i] = cdis;
  }
  float medianDis = findMedian(medianFilter, windowSize);

  if (medianDis > 20) {
        //Serial.println("Ultrasound Reading Distance Out Of Range.");
        //Serial.println(""); 
  }
  else {
      float servoMotorMappedInput = map(medianDis, 0, 20, 0, 180);
      servoMotor.write(servoMotorMappedInput);
      //Serial.println(servoMotorMappedInput);
      delay(15);
  }

}


void readIRSensor(){
  float medianFilter[windowSize];
  for (int i = 0; i < windowSize; i++){
    float volts = analogRead(sharpIRInput) *(5.0 / 1024.0);
    float Dis = 4.4627*volts*volts-19.378*volts+24.957;
    medianFilter[i] = Dis;
  }
  float medianDis = findMedian(medianFilter, windowSize);

  speed_req = constrain(map(int(medianDis), 0, 20, 60, 255), 0, 255);

  digitalWrite(dcMotorIn1, HIGH);
  digitalWrite(dcMotorIn2, LOW);
  
  
  if((millis()-lastMilli) >= LOOPTIME)   {                                    // enter tmed loop
     lastMilli = millis();
     getMotorData(); // calculate speed, volts and Amps
     PWM_val = updatePid(PWM_val, speed_req, speed_act); // compute PWM value
     analogWrite(dcMotorEn, PWM_val); // send PWM to motor
  }
}

void readPotentiometerSensor(){
  float medianFilter[windowSize];
  for (int i = 0; i < windowSize; i++){
    float volts = analogRead(potentiometerOutput) *(5.0 / 1024.0);
    float resis = (2.9263*volts*volts-22.403*volts+97.623)*volts;
    medianFilter[i] = resis;
  }
  float medianResis = findMedian(medianFilter, windowSize);

  float angleStepper = medianResis-currentStepperAngle; 
  int steps = int((angleStepper/0.9))  % stepsPerRevolution;
  currentStepperAngle = medianResis;
  
  stepperMotor.step(steps);
}


void processSwitch(int *sw1Input, int *sw2Input) {
  if ((*sw1Input < 1000) && (*sw2Input < 1000)) {
    *sw1Input = 0;
    *sw2Input = 0;
  }
  else if (*sw1Input < *sw2Input) {
    *sw1Input = 0;
    *sw2Input = 1;
  }
  else if (*sw2Input < *sw1Input) {
    *sw1Input = 1;
    *sw2Input = 0;
  }
  else {
    *sw1Input = 1;
    *sw2Input = 1;
  }
    
}

void readSwitch(int *sw1, int *sw2) {
  int counter1 = 0;
  int counter2 = 0;
  int threshold = windowSize / 2 + 2;
  for (int i = 0; i < windowSize; i++){

    *sw1 = analogRead(sw1Input);
    *sw2 = analogRead(sw2Input);
    processSwitch(sw1, sw2);
    counter1 += *sw1;
    counter2 += *sw2;
  }
  if (counter1 >= threshold) { *sw1 = 1;} else { *sw1 = 0;}
  if (counter2 >= threshold) { *sw2 = 1;} else { *sw2 = 0;}
  }
