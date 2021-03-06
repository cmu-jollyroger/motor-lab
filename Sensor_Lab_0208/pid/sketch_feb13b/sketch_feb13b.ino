// Test MD03a / Pololu motor with encoder
// speed control (PI), V & I display
// Credits:
//   Dallaby   http://letsmakerobots.com/node/19558#comment-49685
//   Bill Porter  http://www.billporter.info/?p=286
//   bobbyorr (nice connection diagram) http://forum.pololu.com/viewtopic.php?f=15&t=1923

#include <Encoder.h>

#define InA1            6                      // INA motor pin
#define InB1            7                      // INB motor pin 
#define PWM1            5                       // PWM motor pin
#define encodPinA1      2                       // encoder A pin
#define encodPinB1      3                       // encoder B pin

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
float Kp =   .4;                                // PID proportional control Gain
float Kd =    1;                                // PID Derivitave control gain

Encoder myEnc(encodPinA1, encodPinB1);

void setup() {
 analogReference(EXTERNAL);                            // Current external ref is 3.3V
 Serial.begin(9600);
 pinMode(InA1, OUTPUT);
 pinMode(InB1, OUTPUT);
 pinMode(PWM1, OUTPUT); 
 for(int i=0; i<NUMREADINGS; i++)   readings[i] = 0;  // initialize readings to 0

 analogWrite(PWM1, PWM_val);
 digitalWrite(InA1, LOW);
 digitalWrite(InB1, HIGH);
}

void loop() {
 getParam();                                                                 // check keyboard
 if((millis()-lastMilli) >= LOOPTIME)   {                                    // enter tmed loop
   lastMilli = millis();
   getMotorData();                                                           // calculate speed, volts and Amps
   PWM_val= updatePid(PWM_val, speed_req, speed_act);                        // compute PWM value
   analogWrite(PWM1, PWM_val);                                               // send PWM to motor
 }
 printMotorInfo();                                                           // display data
}

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

void printMotorInfo()  {                                                      // display data
 if((millis()-lastMilliPrint) >= 500)   {                     
   lastMilliPrint = millis();
   Serial.print("SP:");             Serial.print(speed_req);  
   Serial.print("  RPM:");          Serial.print(speed_act);
   Serial.print("  PWM:");          Serial.print(PWM_val);             
 }
}

int getParam()  {
char param, cmd;
 if(!Serial.available())    return 0;
 delay(10);                  
 param = Serial.read();                              // get parameter byte
 if(!Serial.available())    return 0;
 cmd = Serial.read();                                // get command byte
 Serial.flush();
 switch (param) {
   case 'v':                                         // adjust speed
     if(cmd=='+')  {
       speed_req += 20;
       if(speed_req>400)   speed_req=400;
     }
     if(cmd=='-')    {
       speed_req -= 20;
       if(speed_req<0)   speed_req=0;
     }
     break;
   case 's':                                        // adjust direction
     if(cmd=='+'){
       digitalWrite(InA1, LOW);
       digitalWrite(InB1, HIGH);
     }
     if(cmd=='-')   {
       digitalWrite(InA1, HIGH);
       digitalWrite(InB1, LOW);
     }
     break;
   case 'o':                                        // user should type "oo"
     digitalWrite(InA1, LOW);
     digitalWrite(InB1, LOW);
     speed_req = 0;
     break;
   default: 
     Serial.println("???");
   }
}

int digital_smooth(int value, int *data_array)  {    // remove signal noise
static int ndx=0;                                                         
static int count=0;                          
static int total=0;                          
 total -= data_array[ndx];               
 data_array[ndx] = value;                
 total += data_array[ndx];               
 ndx = (ndx+1) % NUMREADINGS;                                
 if(count < NUMREADINGS)      count++;
 return total/count;
}
