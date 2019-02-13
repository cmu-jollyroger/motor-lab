/*
* Carnegie Mellon University Mechatronics 2019: ShipBot Team A
* Motor Lab
* Authors: David Bang (@dbang)
* Fiona Li (@siling1)
* Sara Misra (@saram1)
* Haowen Shi (@haowensh)
* Bo Tian (@btian1)
*/

import controlP5.*; // import controlP5 library
import processing.serial.*;


ControlP5 controlP5;
Serial myPort;

Textlabel myTextlabelA, myTextlabelB, myTextlabelC, myTextlabelD, myTextlabelE, myTextlabelF, myTextlabelG, myTextlabelH;
Textlabel myTextlabelI, myTextlabelJ, myTextlabelK, myTextlabelL, myTextlabelM; //<>// //<>//
 

boolean sensorMode;

void setup() {
 size(790,560);
 
 controlP5 = new ControlP5(this);
 String portName = Serial.list()[0];
 
 println(Serial.list()); 

 myPort = new Serial(this, portName, 9600);
 
 // description : a toggle can have two states, true and false
 // where true has the value 1 and false is 0.
 // parameters : name, default value (boolean), x, y, width, height
 controlP5.addToggle("Sensor Mode/GUI Mode Toggle",true,10,10,50,50);
 sensorMode = true;
 
 // description : a slider is either used horizontally or vertically.
 // width is bigger, you get a horizontal slider
 // height is bigger, you get a vertical slider. 
 // parameters : name, minimum, maximum, default value (float), x, y, width, height
 myTextlabelB = controlP5.addTextlabel("label1")
                    .setText("Stepper Motor")
                    .setPosition(20, 90)
                    .setColorValue(0xffffffff)
                    .setFont(createFont("Georgia",15));
 
 //controlP5.addSlider("Motor Angle",0,180,90,150,90,250,20);
 
 controlP5.addTextfield("Stepper Input")
     .setPosition(170,90)
     .setSize(220,20)
     .setFont(createFont("Georgia",15))
     .setFocus(true)
     .setColor(color(255,255,255));
 
 
 myTextlabelC = controlP5.addTextlabel("label2")
                    .setText("Stepper Angle: " + "0")
                    .setPosition(470, 90)
                    .setColorValue(0xffffffff)
                    .setFont(createFont("Georgia",15));
                    
                    
 controlP5.addButton("Stepper Go")     
    .setPosition(650, 80)  
    .setSize(80, 40)     
    .setFont(createFont("Georgia",10));
    
    
    
  myTextlabelD = controlP5.addTextlabel("label3")
                    .setText("RC Servo Motor")
                    .setPosition(20, 140)
                    .setColorValue(0xffffffff)
                    .setFont(createFont("Georgia",15));
 
 controlP5.addSlider("RC Motor Angle",0,180,90,150,140,250,20);
 
 myTextlabelE = controlP5.addTextlabel("label4")
                    .setText("Servo Angle: " + "0")
                    .setPosition(470, 140)
                    .setColorValue(0xffffffff)
                    .setFont(createFont("Georgia",15));
                    
                    
 controlP5.addButton("Servo Go")     //"blue" is the name of button
    .setPosition(650, 130)  //x and y coordinates of upper left corner of button
    .setSize(80, 40)      //(width, height)
    .setFont(createFont("Georgia",10));
    
    
    
 myTextlabelF = controlP5.addTextlabel("label5")
                    .setText("DC Motor Pos")
                    .setPosition(20, 190)
                    .setColorValue(0xffffffff)
                    .setFont(createFont("Georgia",15));
  
 
  controlP5.addTextfield("DC Angle")
     .setPosition(170,190)
     .setSize(220,20)
     .setFont(createFont("Georgia",15))
     .setFocus(true)
     .setColor(color(255,255,255));
 
 myTextlabelG = controlP5.addTextlabel("label6")
                    .setText("DC Motor Angle: " + "0")
                    .setPosition(470, 190)
                    .setColorValue(0xffffffff)
                    .setFont(createFont("Georgia",15));
                    
                    
 controlP5.addButton("DC Angle Go")     //"blue" is the name of button
    .setPosition(650, 180)  //x and y coordinates of upper left corner of button
    .setSize(80, 40)      //(width, height)
    .setFont(createFont("Georgia",10));
    
    
    
 myTextlabelH = controlP5.addTextlabel("label7")
                    .setText("DC Motor Vel")
                    .setPosition(20, 240)
                    .setColorValue(0xffffffff)
                    .setFont(createFont("Georgia",15));
    
   controlP5.addTextfield("DC Velocity")
     .setPosition(170,240)
     .setSize(220,20)
     .setFont(createFont("Georgia",15))
     .setFocus(true)
     .setColor(color(255,255,255));
 
 myTextlabelI = controlP5.addTextlabel("label8")
                    .setText("Motor Velocity: " + "0")
                    .setPosition(470, 240)
                    .setColorValue(0xffffffff)
                    .setFont(createFont("Georgia",15));
                    
                    
 controlP5.addButton("DC Vel Go")     //"blue" is the name of button
    .setPosition(650, 230)  //x and y coordinates of upper left corner of button
    .setSize(80, 40)      //(width, height)
    .setFont(createFont("Georgia",10));
 
 

                      
  myTextlabelA = controlP5.addTextlabel("label")
                    .setText("Motor Controller GUI")
                    .setPosition(200,00)
                    .setColorValue(0xffffffff)
                    .setFont(createFont("Georgia",35));
                    
                    
 //Sensor Inputs
   
 myTextlabelJ = controlP5.addTextlabel("label9")
                .setText("Sensor Values")
                .setPosition(250,300)
                .setColorValue(0xffffffff)
                .setFont(createFont("Georgia",35));
                
                
 myTextlabelK = controlP5.addTextlabel("label10")
                    .setText("Potentiometer Value (in degrees): " + "0")
                    .setPosition(20, 350)
                    .setColorValue(0xffffffff)
                    .setFont(createFont("Georgia",15));
   
   
 myTextlabelL = controlP5.addTextlabel("label11")
                    .setText("Ultrasound Sensor Value (in inches): " + "0")
                    .setPosition(20, 380)
                    .setColorValue(0xffffffff)
                    .setFont(createFont("Georgia",15));
                    
 myTextlabelM = controlP5.addTextlabel("label12")
                    .setText("IR Sensor Value (in inches): " + "0")
                    .setPosition(20, 410)
                    .setColorValue(0xffffffff)
                    .setFont(createFont("Georgia",15));
 
}
 
void draw() { 
 background(10, 20, 20); 
 
}

void serialEvent  (Serial myPort) {
  println("SerialEvent Received");
  String update  =  myPort.readStringUntil ( '\n' );
  
  println("Update Received " + update);
  if (update.indexOf("UltraSound:") != -1){
    String value = update.substring(update.lastIndexOf(':'));
    myTextlabelL.setText("Ultrasound Sensor Value(in inches): " + value);
  }
  else if (update.indexOf("IR:") != -1) {
    String value = update.substring(update.lastIndexOf(':'));
    myTextlabelM.setText("IR Sensor Value(in inches): " + value);
  }
  else if (update.indexOf("Potentiometer:") != -1) {
    String value = update.substring(update.lastIndexOf(':'));
    myTextlabelK.setText("Potentiometer Value(in degrees): " + value);
  }
  else if (update.indexOf("ServoAngle:") != -1) {
    String value = update.substring(update.lastIndexOf(':'));
    myTextlabelE.setText("Servo Angle: " + value);
  }
  else if (update.indexOf("DC:") != -1) {
    String value = update.substring(update.lastIndexOf(':'));
    myTextlabelC.setText("Stepper Angle: " + value);
  }
  else if (update.indexOf("DCAngle:") != -1) {
    String value = update.substring(update.lastIndexOf(':'));
    myTextlabelG.setText("DC Angle: " + value);
  }
} 


void controlEvent(ControlEvent theEvent) {
 if(theEvent.isController()) { 
 
   print("control event from : "+theEvent.getController().getName());
   println(", value : "+theEvent.getController().getValue());
   
   if(theEvent.getController().getName()=="Sensor Mode/GUI Mode Toggle") {
     if(theEvent.getController().getValue()==1) {
       println("Sending Sensor Command Toggle");
       myPort.write("Sensor Command:" + "true" + '\n');
       sensorMode = true;
     }
     else {
       myPort.write("Sensor Command:" + "false" + '\n');
       sensorMode = false;
     }
   }
   if (sensorMode == false) {
   
     if(theEvent.getController().getName()=="Stepper Go") {
       String stepperInput = controlP5.get(Textfield.class,"Stepper Input").getText();
       println("Sending Stepper Command " + stepperInput); //<>// //<>//

       myPort.write("Stepper Command:" + stepperInput + '\n');
     }
   
     if(theEvent.getController().getName()=="Servo Go") { //<>// //<>//
       float servoInput = controlP5.get(Slider.class,"RC Motor Angle").getValue();
       println("About to send Servo Command " + servoInput);
       myPort.write("Servo Command:" + servoInput + '\n');
       println("Sent Servo Command");
     }
   
     if(theEvent.getController().getName()=="DC Angle Go") {
       String dcAngleInput = controlP5.get(Textfield.class,"DC Angle").getText();
       println("Sending DC Angle Command " + dcAngleInput);

       myPort.write("DCAngle Command:" + dcAngleInput + '\n');
     }
     
     if(theEvent.getController().getName()=="DC Vel Go") {
       String dcVelocityInput = controlP5.get(Textfield.class,"DC Velocity").getText();
       println("Sending DC Vel Command " + dcVelocityInput);

       myPort.write("DCVelocity Command:" + dcVelocityInput + '\n');
     }
   }
 }
 
}
