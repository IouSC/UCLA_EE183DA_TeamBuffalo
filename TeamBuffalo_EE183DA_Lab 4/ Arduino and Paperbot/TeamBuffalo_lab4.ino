
/*
  Wireless Servo Control, with ESP as Access Point

  Usage:
    Connect phone or laptop to "ESP_XXXX" wireless network, where XXXX is the ID of the robot
    Go to 192.168.4.1.
    A webpage with four buttons should appear. Click them to move the robot.

  Installation:
    In Arduino, go to Tools > ESP8266 Sketch Data Upload to upload the files from ./data to the ESP
    Then, in Arduino, compile and upload sketch to the ESP

  Requirements:
    Arduino support for ESP8266 board
      In Arduino, add URL to Files > Preferences > Additional Board Managers URL.
      See https://learn.sparkfun.com/tutorials/esp8266-thing-hookup-guide/installing-the-esp8266-arduino-addon

    Websockets library
      To install, Sketch > Include Library > Manage Libraries... > Websockets > Install
      https://github.com/Links2004/arduinoWebSockets

    ESP8266FS tool
      To install, create "tools" folder in Arduino, download, and unzip. See
      https://github.com/esp8266/Arduino/blob/master/doc/filesystem.md#uploading-files-to-file-system

  Hardware:
    NodeMCU Amica DevKit Board (ESP8266 chip)
    Motorshield for NodeMCU
    2 continuous rotation servos plugged into motorshield pins D1, D2
    Ultra-thin power bank
    Paper chassis

*/
//library for basic arduino function
#include <Arduino.h>

//library for ESP8266 WiFi function
#include <BearSSLHelpers.h>
#include <CertStoreBearSSL.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiAP.h>
#include <ESP8266WiFiGeneric.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266WiFiScan.h>
#include <ESP8266WiFiSTA.h>
#include <ESP8266WiFiType.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <WiFiClientSecureAxTLS.h>
#include <WiFiClientSecureBearSSL.h>
#include <WiFiServer.h>
#include <WiFiServerSecure.h>
#include <WiFiServerSecureAxTLS.h>
#include <WiFiServerSecureBearSSL.h>
#include <WiFiUdp.h>

//library for lidar sensor
#include <VL53L0X.h>

//library for servo
#include <Servo.h>

//library for I2C
#include <Wire.h>

//Magnetormeter Setup
#define   MPU9250_ADDRESS             0x68
#define   MAG_ADDRESS                 0x0C

#define   GYRO_FULL_SCALE_250_DPS     0x00
#define   GYRO_FULL_SCALE_500_DPS     0x08
#define   GYRO_FULL_SCALE_1000_DPS    0x10
#define   GYRO_FULL_SCALE_2000_DPS    0x18

#define   ACC_FULL_SCALE_2_G          0x00
#define   ACC_FULL_SCALE_4_G          0x08
#define   ACC_FULL_SCALE_8_G          0x10
#define   ACC_FULL_SCALE_16_G         0x18

#define SDA_PORT 14
#define SCL_PORT 12

#define   MPU9250Mmode                0x02

// Define rad and deg convertion
#define   DEG_2_RAD           PI/180
#define   RAD_2_DEG           180/PI

// Define declination angle on UCLA campus [deg]
#define   DECLINATION_ANGLE   (11 + (56/60))

//Constant for Servo
const int SERVO_LEFT = D1;
const int SERVO_RIGHT = D2;
int servo_left_ctr = 90;
int servo_right_ctr = 90;

//Magnetometer Calibration
float     MPU9250biases[3] = {78.25,29.5,-3}; 
float     MPU9250scales[3] = {0.835,0.845,0.84}; 

//Sensor Object
VL53L0X sensor;
VL53L0X sensor2;

//Servo Object
Servo servo_left;
Servo servo_right;

// This function read Nbytes bytes from I2C device at address Address.
// Put read bytes starting at register Register in the Data array.
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();

  // Read Nbytes
  Wire.requestFrom(Address, Nbytes);
  uint8_t index = 0;
  while (Wire.available())
    Data[index++] = Wire.read();
}

// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

// Movement Functions
void drive(int left, int right) {
  servo_left.write(left);
  servo_right.write(right);
}

void stop() {
  servo_left.detach();
  servo_right.detach();
  //drive(servo_left_ctr, servo_right_ctr);
  LED_OFF;
}

void forward() {
  servo_left.attach(SERVO_LEFT);
  servo_right.attach(SERVO_RIGHT);
  drive(107, 63);
}

void backward() {
  servo_left.attach(SERVO_LEFT);
  servo_right.attach(SERVO_RIGHT);
  drive(65, 106);
}

void left() {
  servo_left.attach(SERVO_LEFT);
  servo_right.attach(SERVO_RIGHT);
  drive(65, 64);
}

void right() {
  servo_left.attach(SERVO_LEFT);
  servo_right.attach(SERVO_RIGHT);
  drive(105, 114);
}

//Setup pins
void setupPins() {
  // setup Serial, LEDs and Motors
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);    //Pin D0 is LED
  LED_OFF;                     //Turn off LED

  servo_left.attach(SERVO_LEFT);
  servo_right.attach(SERVO_RIGHT);

  pinMode(D3, OUTPUT);
  pinMode(D4, OUTPUT);
  digitalWrite(D7, LOW);
  digitalWrite(D8, LOW);

  delay(500);
  Wire.begin(SDA_PORT, SCL_PORT);

  digitalWrite(D3, HIGH);
  delay(150);
  Serial.println("00");

  sensor.init(true);
  Serial.println("01");
  delay(100);
  sensor.setAddress((uint8_t)22);

  digitalWrite(D4, HIGH);
  delay(150);
  sensor2.init(true);
  Serial.println("03");
  delay(100);
  sensor2.setAddress((uint8_t)25);
  Serial.println("04");

  Serial.println("addresses set");
  Serial.println ("I2C scanner. Scanning ...");
  byte count = 0;

  for (byte i = 1; i < 120; i++)
  {

    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0)
    {
      Serial.print ("Found address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);
      Serial.println (")");
      count++;
      delay (1);  // maybe unneeded?
    } // end of good response
  } // end of for loop
  Serial.println ("Done.");
  Serial.print ("Found ");
  Serial.print (count, DEC);
  Serial.println (" device(s).");

  delay(3000);
}

//WiFi Setup
WiFiClient client;
WiFiServer server(80);

//Setup
void setup() {
  setupPins();
  //setup for the WiFi acesspoint
  WiFi.mode(WIFI_AP);
  WiFi.softAP("crazymare");
  server.begin();

  for (uint8_t t = 4; t > 0; t--) {
    Serial.printf("[SETUP] BOOT WAIT %d...\n", t);
    Serial.flush();
    LED_ON;
    delay(500);
    LED_OFF;
    delay(500);
  }
  LED_ON;
  //    setupSTA(sta_ssid, sta_password);
  //    setupAP(ap_ssid, ap_password);
  LED_OFF;
  
  stop();
  // Arduino initializations
  Wire.begin(SDA_PORT, SCL_PORT);

  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS, 0x37, 0x02);

  // Request first magnetometer single measurement
  I2CwriteByte(MAG_ADDRESS, 0x0A, 0x01);
}

//main
void loop() {
  client = server.available();//check if there is any client has data ready to read

  if (client) {
    
    //loop as long as MATLAB is connected
    while (client.connected()) {

      //loop if there is no command to read
      while (!client.available()) {
        
        //break when MATLAB disconnect
        if (!client.connected())break;
        delay(1);
      }
      
      //Receive command and processing delay
      unsigned long start_time = millis();
      unsigned int command_time_floor = (unsigned int)client.read();
      unsigned int command_time_mod = (unsigned int)client.read();
      unsigned long command_time = (unsigned long)(command_time_floor * 256 + command_time_mod);

      
      unsigned long reverse_time = 20;

      //Run the assigned command
      char command = (unsigned int)client.read();
      switch (command)
      {
        case 'F':
          {
            while (((millis() - start_time)) < command_time) {
              forward();
              yield();
            }

            unsigned long counter = millis();
            while(((millis() - counter)) < reverse_time)
            {
              backward();
              yield();
              }
            stop();
            break;
          }
        case 'B':
          {
            while (((millis() - start_time)) < command_time) 
            {
              backward();
              yield();
            }

            unsigned long counter = millis();
            while(((millis() - counter)) < reverse_time)
            {
              forward();
              yield();
              }
            stop();
            break;
          }
        case 'L':
          {
            while (((millis() - start_time)) < command_time) {
              left();
              yield();
            }
            stop();
            break;
          }
        case 'R':
          {
            while (((millis() - start_time)) < command_time) {
              right();
              yield();
            }
            stop();
            break;
          }
        case 'X':
          {
            unsigned int front_sensor_reading = sensor.readRangeSingleMillimeters();
            byte x[2] = {highByte(front_sensor_reading), lowByte(front_sensor_reading)};
            client.write(x, 2);
            Serial.println("X SENT");
            client.flush();
            break;
          }
        case 'Y':
          {
            unsigned int right_sensor_reading = sensor2.readRangeSingleMillimeters();
            byte y[2] = {highByte(right_sensor_reading), lowByte(right_sensor_reading)};
            client.write(y, 2);
            Serial.println("Y SENT");
            client.flush();
            break;
          }
        case 'H':
          {
            /* ::::::::::  Magnetometer :::::::::: */
            // Request first magnetometer single measurement
            I2CwriteByte(MAG_ADDRESS, 0x0A, 0x01);

            // Read register Status 1 and wait for the DRDY: Data Ready
            uint8_t ST1;
            do
            {
              I2Cread(MAG_ADDRESS, 0x02, 1, &ST1);
            }
            while (!(ST1 & 0x01));

            // Read magnetometer data
            uint8_t Mag[7];
            I2Cread(MAG_ADDRESS, 0x03, 7, Mag);

            // Create 16 bits values from 8 bits data

            // Magnetometer
            int16_t mx = (Mag[1] << 8 | Mag[0]);
            int16_t my = (Mag[3] << 8 | Mag[2]);
            int16_t mz = (Mag[5] << 8 | Mag[4]);
            mx = MPU9250scales[0] * (mx - MPU9250biases[0]);
            my = MPU9250scales[1] * (my - MPU9250biases[1]);
            mz = MPU9250scales[2] * (mz - MPU9250biases[2]);
            float heading = atan2(mx, my);

            // Once you have your heading, you must then add your 'Declination Angle',
            // which is the 'Error' of the magnetic field in your location. Mine is 0.0404
            // Find yours here: http://www.magnetic-declination.com/

            // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
            /*
              From http://www.magnetic-declination.com/, on UCLA campus we have a declination of +11deg 56'
              which is equivalent to +(11+56/60)deg
              which is equivalent to +((11+56/60)*(pi/180))rad
            */
            float declinationAngle = DECLINATION_ANGLE * DEG_2_RAD;
            heading += declinationAngle;

            // Correct for when signs are reversed.
            if (heading < 0)
              heading += 2 * PI;

            // Check for wrap due to addition of declination.
            if (heading > 2 * PI)
              heading -= 2 * PI;

            // Convert radians to degrees for readability.
            float headingDegrees = heading * RAD_2_DEG;

            unsigned int heading_transfer = (unsigned int)headingDegrees;
            byte h[2] = {highByte(heading_transfer), lowByte(heading_transfer)};
            client.write(h, 2);
            Serial.println("H SENT");
            client.flush();
            break;
          }
        case 'S':
          {
            stop();
            break;
          }
        default:
          {
            break;
          }
      }
    }
  }
}


