// Include the Arduino Stepper Library
#include <Stepper.h>

// Number of steps per output rotation
const int stepsPerRevolution = 200;

// Create Instance of Stepper library
Stepper Rail_stepper(stepsPerRevolution, 23, 25, 27, 29);
Stepper Base_stepper(stepsPerRevolution, 4, 5, 6, 7);

void setup()
{
  // set the speed at 60 rpm:
  Rail_stepper.setSpeed(60);
  Base_stepper.setSpeed(60);
  // initialize the serial port:
  Serial.begin(9600);
}

void loop() 
{
  delay(1000);
//  //step one revolution in one direction:
//  Serial.println("clockwise");
  Rail_stepper.step(200);//90 degree
  //Base_stepper.step(200);
    delay(2000);
//  //step one revolution in one direction:
//  Serial.println("counterclockwise");
  Rail_stepper.step(-200);//90 degree
  //Base_stepper.step(-200);
  delay(2000);
}
