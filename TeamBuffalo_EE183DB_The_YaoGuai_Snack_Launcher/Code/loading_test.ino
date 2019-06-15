// Arduino Library
#include <Stepper.h>
#include <boarddefs.h>
#include <IRremote.h>
#include <IRremoteInt.h>
#include <ir_Lego_PF_BitStreamEncoder.h>

// IR
const byte pinIR(2);
volatile byte state(LOW);

// REMOTE
const byte RECV_PIN(4);
IRrecv irrecv(RECV_PIN);
decode_results results;

// Number of steps per output rotation
const int stepsPerRevolution(200);

// Create Instance of Stepper library
Stepper load_stepper(stepsPerRevolution,8,9,10,11);

void setup()
{
  // set the speed at 60 rpm:
  load_stepper.setSpeed(60);
  
  // IR interrupt
  pinMode(pinIR, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinIR), load_itr, HIGH);
  
  // initialize the serial port:
  Serial.begin(9600);

  // REMOTE receiver
  irrecv.enableIRIn(); // Start the receiver
}

void loop() 
{
  if (irrecv.decode(&results)){
    Serial.println(results.value);
    if (state == LOW) {
      load_stepper.step(1);
    } else if (state == HIGH) {
      irrecv.resume();
    }
  }
}

void load_itr() {
  state = !state;
}

