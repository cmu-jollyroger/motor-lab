/*
* Carnegie Mellon Univeristy Mechatronics 2019: ShipBot Team A
* Sensor Lab
* Authors: David Bang (@dbang)
* Fiona Li (@siling1)
* Sara Misra (@saram1)
* Haowen Shi (@haowensh)
* Bo Tian (@btian1)
*/

#include <Servo.h>

Servo servoMotor;  // create servo object to control a servo


// Digital Pin Assignments
const int trigOutput = 2;
const int echoInput = 3;
const int servoMotorOutput = 9;

// Analog Pin Assignments
const int sharpIRInput = 0;
const int potentiometerOutput = 1;
const int sw1Input = A4;
const int sw2Input = A5;

const int windowSize = 20;

float USS_result = 0;
char last;
boolean plotMode = false;
//boolean plotMode = true;

int sw1 = 1;
int sw2 = 1;
void setup() {

  //Pin Setup
  pinMode(trigOutput, OUTPUT);
  pinMode(echoInput, INPUT);
  pinMode(sharpIRInput, INPUT);
  pinMode(potentiometerOutput, OUTPUT);
  Serial.begin(9600);
  servoMotor.attach(servoMotorOutput);
  
  //Switch Pin Setup
  pinMode(sw1Input, INPUT);
  pinMode(sw2Input, INPUT);

  //Terminal Interface
  Serial.println("CMU Mechatronics ShipBot Team A: Sensors Lab");
  prettyPrintHelper();
  printHowToUse();
}

void printHowToUse() {
 Serial.println("List of commands is specified below");
 Serial.println(" h: show help instructions again");
 Serial.println(" p: Return Angle of Potentiometer In Degrees");
 Serial.println(" i: Return Distance of IR Sensor In Inches");
 Serial.println(" u: Return Distance of UltraSound Sensor in Inchines");
 Serial.println(" d: Demos all 3 sensors continously 20 times");
 Serial.println("To continue send a character followed by a newline.");
 prettyPrintHelper();
}

void prettyPrintHelper(){
  for (int i = 0; i < 100; i++) {
    Serial.print("=");
    }
    Serial.println();
}

void loop() {
  readSwitch(&sw1, &sw2);
  if ((sw1 == 0) && (sw2 == 0)) {
    readUltraSoundSensor();
  } 
  else if ((sw1 == 0) && (sw2 == 1)) {
    readIRSensor();
  }
  else if ((sw1 == 1) && (sw2 == 0)) {
    readPotentiometerSensor();
  }
  else {}
  
  
//  if (plotMode) {
//      readUltraSoundSensor();
//      readIRSensor();
//      readPotentiometerSensor();
//  }
//  else if (Serial.available()) {
//    char readIn = Serial.read();
//    if ((int) readIn == 10) {
//      Serial.println("Input is");
//      Serial.println(last);
//      modeSwitch(last);
//    }
//    last = readIn;
//  }
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
  
  if (!plotMode){
    Serial.println("Ultrasound Reading"); 
    Serial.print("Distance: ");
    Serial.print(medianDis); 
    Serial.println(" inches");
    Serial.println(""); 
  }
  else {
    Serial.print(medianDis);
    Serial.print(",");
  }
  if (medianDis > 20) {
        Serial.println("Ultrasound Reading Distance Out Of Range.");
        Serial.println(""); 
  }
  else {
      float servoMotorMappedInput = map(medianDis, 0, 20, 0, 180);
      servoMotor.write(servoMotorMappedInput);
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
  if (!plotMode){
    Serial.println("IR Reading"); 

    Serial.print("Distance: ");
    Serial.print(medianDis); 
    Serial.println(" inches");
    Serial.println(""); 
  }
  else {
    Serial.print(medianDis);
    Serial.print(",");
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
  if (!plotMode) {
    Serial.println("Potentiometer Reading");
    Serial.print("Angle: ");
    Serial.print(medianResis);
    Serial.println(" degrees");
    Serial.println("");
  }
  else {
    Serial.println(medianResis);
  }
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
//    sw1 = digitalRead(sw1Input);
//    sw2 = digitalRead(sw2Input);
    *sw1 = analogRead(sw1Input);
    *sw2 = analogRead(sw2Input);
    processSwitch(sw1, sw2);
    counter1 += *sw1;
    counter2 += *sw2;
//    Serial.println("==========i:");
//    Serial.println(i);
//    Serial.println("==========");
//    Serial.print("counter 1: ");
//    Serial.println(counter1);
//    Serial.print("counter 2: ");
//    Serial.println(counter2);
//    Serial.print("Switch 1: ");
//    Serial.println(sw1);
//    Serial.print("Switch 2: ");
//    Serial.println(sw2);
//    Serial.print("analogRead(sw1Input) 1: ");
//    Serial.println(analogRead(sw1Input));
//    Serial.print("analogRead(sw2Input) 2: ");
//    Serial.println(analogRead(sw2Input));
//    Serial.println("==========");
  }
  if (counter1 >= threshold) { *sw1 = 1;} else { *sw1 = 0;}
  if (counter2 >= threshold) { *sw2 = 1;} else { *sw2 = 0;}
//  Serial.print("threshold: ");
//  Serial.println(threshold);
//  Serial.print("windowSize: ");
//  Serial.println(windowSize);
//  Serial.print("counter 1: ");
//  Serial.println(counter1);
//  Serial.print("counter 2: ");
//  Serial.println(counter2);
  Serial.print("Switch 1: ");
  Serial.println(*sw1);
  Serial.print("Switch 2: ");
  Serial.println(*sw2);
  Serial.println("==========");
  }

void modeSwitch(char input) {
  plotMode = false;
  switch (input) {
    case 'h':
      printHowToUse();
      break;
    case 'p':
      readPotentiometerSensor();  
      prettyPrintHelper();
      break;
    case 'i':
      readIRSensor();
      prettyPrintHelper();
      break;
    case 'u':
      readUltraSoundSensor();
      prettyPrintHelper();
      break;
//    case 's':
//      while(1){
//        readSwitch(&sw1, &sw2);
//        delay(50);
//      }
//      break;
    case 'd':
//      int i = 0;
      while(1) {
        readUltraSoundSensor();
        readIRSensor();
        readPotentiometerSensor();  
        prettyPrintHelper();
        delay(500);
//        i++;
      }
      break;
    default:
      Serial.println("Invalid Input.");
      prettyPrintHelper();
  }
}
