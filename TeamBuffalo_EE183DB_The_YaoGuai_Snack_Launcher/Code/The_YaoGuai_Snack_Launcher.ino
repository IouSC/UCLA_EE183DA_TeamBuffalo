// Include the Arduino Stepper Library

#include <Stepper.h>
#include <Servo.h>
// Include the Arduino SoftwareSerial Library
#include <boarddefs.h>
#include <IRremote.h>
#include <IRremoteInt.h>
#include <ir_Lego_PF_BitStreamEncoder.h>
#include <SoftwareSerial.h>
#include "TFMini.h"

Servo LW_L;
Servo LW_R;

// PWM
#define PWM_MIN 1000
#define PWM_MAX 2000

// IR
const byte pin_IR(2);
volatile byte state_IR(LOW);

//LS
const byte pin_LS(3);
volatile byte state_ls(LOW);

//REMOTE
const byte RECV_PIN(47);
IRrecv irrecv(RECV_PIN);
decode_results results;

// Number of steps per output rotation
const int stepsPerRevolution(200);
//TFMini tfmini;

//communication with openCV
String strX = "";         
bool keepX = false;
bool stringComplete = false;
bool load = false;
bool aim = false; 
bool shooting = false; 
unsigned long time_now;
unsigned long time_pre;
double g = 9.8;
double h = 0.2;
double h_launch = 0.1638; 
double l = 0.14707;  
double r = 0.02;
// webcam reading
int x_difference = 0;

// Create Instance of Stepper library
Stepper base_stepper(stepsPerRevolution, 22,24,26, 28);
Stepper rail_stepperL(stepsPerRevolution, 23, 25, 27, 29);
Stepper rail_stepperR(stepsPerRevolution, 33, 35, 37, 39);
Stepper loader_stepper(stepsPerRevolution, 32, 34, 36, 38);
//SoftwareSerial tfmini_channel(50, 51);                         // Uno RX (TFMINI TX), Uno TX (TFMINI RX)

int senoid_1 = 40; 
int senoid_2 = 41; 
#define pin_LW_L 9
#define pin_LW_R 10

SoftwareSerial tfmini_channel(50, 51);                         // Uno RX (TFMINI TX), Uno TX (TFMINI RX)
TFMini tfmini;

void setup(){
  //set up wheel
  LW_L.attach(pin_LW_L);
  LW_R.attach(pin_LW_R);
  LW_L.writeMicroseconds(1500);
  LW_R.writeMicroseconds(1500);
  
  // initialize the serial port:
  Serial.begin(9600);

  //set up stepper
  base_stepper.setSpeed(60);
  loader_stepper.setSpeed(60);
  rail_stepperR.setSpeed(60);
  rail_stepperL.setSpeed(60);
  
  // set up lidar
  //tfmini_channel.begin(TFMINI_BAUDRATE);
  //tfmini.begin(&tfmini_channel);

  // LS interrupt
  pinMode(pin_LS, INPUT);
  attachInterrupt(digitalPinToInterrupt(pin_LS), LS_itr, HIGH);
  attachInterrupt(digitalPinToInterrupt(pin_IR), IR_itr, HIGH);
  tfmini_channel.begin(TFMINI_BAUDRATE);
  tfmini.begin(&tfmini_channel);

  // REMOTE receiver
  irrecv.enableIRIn(); // Start the receiver

  //sonoid 
  pinMode(senoid_1,OUTPUT);
  pinMode(senoid_2,OUTPUT);  
}

void loop()
{
  int i = 0;
  while(load ==false ){
    if (irrecv.decode(&results)){
      while(i<220){
        rail_stepperR.step(-1);
        rail_stepperL.step(1);
        i = i+1; 
      }
      irrecv.resume();
      load = true;
    }
  }
  delay(500);
  load = false; 
  while(load == false ){
    if (state_IR == LOW) {
      loader_stepper.step(1);
      }else if (state_IR == HIGH) {
        load = true;
      }
    }
  
  delay(500);
  i = 0;
  while(i<220){
    rail_stepperR.step(1);
    rail_stepperL.step(-1);
    i = i+1; 
  }
  Serial.println("end");
//  


  while(aim == false){
    x_difference = 0; 
    receiving_openCV();
    if (stringComplete) {
      Serial.println("X:" + strX);
      if (strX == "")
      {
        Serial.println("SSID:not config");
      }
      x_difference = (int)strX.toInt();
      if(x_difference == 999) {
        shoot();
        aim = true;
       }else{
        if(x_difference != 0) base_stepper.step(x_difference);
       } 
      stringComplete = false;
      Serial.flush();
    } 
  }
  aim = false;
  load = false;
  shooting = false;
  Serial.println("restart");
  
}

void receiving_openCV() {
  strX = ""; 
  while(!Serial.available()){}
  while (Serial.available()) {
    delay(30);
    if (Serial.available() >0)
    {
      char inChar = Serial.read();  //gets one byte from serial buffer
      switch (inChar ){
        case '&':{
          stringComplete = true;
          return;
        }
        default:{
          strX += inChar;
        }
        break;
      }
    }
  }
}

void shoot(){
  delay(3000); 
  Serial.println("here");
  double dist = tfmini.getDistance();
  dist = (dist+3.0263)/1.0538 - 15.5;
  dist = dist/100; 
  Serial.print("dist:");
  Serial.println(dist);
  //if (dist > 6.00){return;}
  double h = 8.0; 
  double theta = 90.0; 
  while (theta > radians(60)){
    double a = -1*sqrt(64.4*(h-2.75))-1*sqrt(64.4*(h-2.75) - 4*16.1*1.25);
    double t = a/-32.2;
    theta = atan(sqrt(64.4*(h-2.75))*t/dist);
    h = h-0.5; 
  }
  Serial.print("h:");
  Serial.println(h);
  

  if (theta < radians(45)){
    theta = radians(45);
  }
  
  double v = sqrt(64.4*(h-2.75))/sin(theta); 
  double w = 30*v/PI/r;
  
  Serial.print("theta:");
  Serial.println(theta);
  
  int final_step = (theta/3.14*180 - 45) * 9.17;
  Serial.print("final_step:");
  Serial.println(final_step);
  
  int i = 0;
  while(i<final_step){
    rail_stepperR.step(-1);
    rail_stepperL.step(1);
    i = i+1; 
  }
  
  double left = 1500; 
  double right = 1500;
  Serial.print("w:");
  Serial.println(w);
  w = 1.2*w;
  if(w < 10395){
    if(w > 4618){left = -0.0175 *w + 1350.2; }
    else{left = -0.0294 *w + 1399.2;}
  }
  else{left = -0.0146 *w + 1325.4; }

  if(w < 9538){
      if(w > 4420){right = 0.0179 *w + 1621.4; }
      else{right = 0.0278 *w + 1580;}
  }
  else{right = 0.0139 *w + 1660;}

  Serial.print("L:");
  Serial.print(left);
  
  Serial.print(" R:");
  Serial.print(right);

  left = constrain(left,PWM_MIN,PWM_MAX);
  right = constrain(right,PWM_MIN,PWM_MAX);
  time_now = millis(); 
  while((millis() - time_now)<3000){
      LW_L.writeMicroseconds(int(left));
      LW_R.writeMicroseconds(int(right));
  }

  while(shooting == false){
    LW_L.writeMicroseconds(int(left));
    LW_R.writeMicroseconds(int(right));
    if( state_ls == HIGH || (millis() - time_now)>4000){
      shooting = true;
      while((millis() - time_pre) <2000){
      digitalWrite(senoid_1,LOW);
      LW_L.writeMicroseconds(1660);
      LW_R.writeMicroseconds(1315);
      }
   }else{
      Serial.print("shoot:");
      digitalWrite(senoid_1,HIGH);
   }
  }  
  LW_L.writeMicroseconds(1500);
  LW_R.writeMicroseconds(1500);
  
  i = 0;
  while(i<final_step){
    rail_stepperR.step(1);
    rail_stepperL.step(-1);
    i = i+1; 
  }
  state_ls = LOW;
  Serial.println("end");
}

void LS_itr(){
  state_ls = !state_ls;
  time_pre = millis();
}

void IR_itr(){
  state_IR = !state_IR;
}
