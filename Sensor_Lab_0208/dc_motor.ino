


#define in1 7
#define in2 6
#define pwm 5


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
  Serial.print("Rotating the DC Motor \n") ;
}
