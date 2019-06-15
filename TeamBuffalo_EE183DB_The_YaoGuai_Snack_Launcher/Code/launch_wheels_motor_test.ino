#include <Servo.h>
Servo launchMotor_R;
Servo launchMotor_L;

#define pinLM_R 9
#define pinLM_L 10

#define uk_min 1000
#define uk_max 2000
#define uk_nplus 1480   //1485 rpm 2100
#define uk_nminus 1440 // 1430 rpm 1270
#define uk_stop (uk_nplus + uk_nminus)/2

unsigned int uk_LM_R = 0;
unsigned int uk_LM_L = 0;
unsigned int LM_RPM = 0;

void setup() {
  Serial.begin(9600);

  launchMotor_R.attach(pinLM_R, uk_min, uk_max);
  launchMotor_L.attach(pinLM_L, uk_min, uk_max);
  
  launchMotor_R.writeMicroseconds(uk_stop);
  launchMotor_L.writeMicroseconds(uk_stop);
}

void loop() {
  if (LM_RPM == 0)
  {
    Serial.println("Desired Launch Wheel RPM?");
  }

  // Get User Input RPM value
  while (Serial.available() <= 0) {}
  LM_RPM = Serial.parseInt();
  
  if (LM_RPM >= 0) {
    Serial.print("Right launch wheel servo input uk_LM_R = ");
    uk_LM_R = -0.021*LM_RPM + 1458.6;
    Serial.println(uk_LM_R);
    
    Serial.print("Left launch wheel servo input uk_LM_L = ");
    uk_LM_L = 0.0215*LM_RPM + 1450.7;
    Serial.println(uk_LM_L);
  }

  Serial.read();

  Serial.println("Desired Launch Wheel RPM?");
  while (Serial.available() <= 0)
  {
    launchMotor_R.writeMicroseconds(uk_LM_R);
    launchMotor_L.writeMicroseconds(uk_LM_L);
  }
}
