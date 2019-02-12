

#include <Encoder.h>
#define in1 7
#define in2 6
#define pwm 5
//#define INPUT_PULLUP
#define ENCODER_USE_INTERRUPTS
Encoder myEnc(25,23);
void setup() {
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(pwm, OUTPUT);
  // initialize the serial port:
  Serial.begin(9600);
}

void loop() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(pwm,60); 
  delay(20);
  Serial.print("Encoder Reading ");
  Serial.println(myEnc.read());
  Serial.println("Rotating the DC Motor \n") ;
}
