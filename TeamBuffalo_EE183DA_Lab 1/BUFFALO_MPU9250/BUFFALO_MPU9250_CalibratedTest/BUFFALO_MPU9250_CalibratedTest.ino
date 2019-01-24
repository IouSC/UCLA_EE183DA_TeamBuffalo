/* EE 183DA Team Buffalo
 * MPU9250 Sensor Calibration Test Code
 * 
 * Calibration code adapted from: kriswiner
 * Original Source: https://github.com/kriswiner/MPU6050/wiki/Simple-and-Effective-Magnetometer-Calibration
 * 
 * Adapted from: stevenvo
 * Original Source: https://github.com/stevenvo/mpuarduino/blob/master/mpuarduino.ino
 * Edited for EE183DA by Nathan Pilbrough
 * 
 * Description: Basic code to test the functionality of the
 * MPU9250 sensor. Refer to the startup guide on CCLE for 
 * more information. NOTE: this is meant to help confirm 
 * communication with the sensor, calibration is still required
 */
#include <Wire.h>

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

#define   SDA_PORT 14
#define   SCL_PORT 12

// MPU9250 Calibration
#define   MPU9250Mmode                0x02
float     MPU9250biases[3] = {79.000000,35.000000,2.000000}; 
float     MPU9250scales[3] = {0.679012,0.833333,3.055556}; 

// Define rad and deg convertion
#define   DEG_2_RAD           PI/180 
#define   RAD_2_DEG           180/PI 

// Define declination angle on UCLA campus [deg]
#define   DECLINATION_ANGLE   (11 + (56/60))

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
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
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

// Initializations
void setup()
{
  // Arduino initializations
  Wire.begin(SDA_PORT,SCL_PORT);
  Serial.begin(115200);

  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
  
  // Request first magnetometer single measurement
  I2CwriteByte(MAG_ADDRESS,0x0A,0x01);
}

long int  cpt = 0;
// Main loop, read and display data
void loop()
{
  /* :::::::::: Counter :::::::::: */
  // Display data counter
//  Serial.print (cpt++,DEC);
//  Serial.print ("\t");
  
  /* ::::::::::  Magnetometer :::::::::: */ 
  // Request first magnetometer single measurement
  I2CwriteByte(MAG_ADDRESS,0x0A,0x01);
  
  // Read register Status 1 and wait for the DRDY: Data Ready
  uint8_t ST1;
  do
  {
    I2Cread(MAG_ADDRESS,0x02,1,&ST1);
  }
  while (!(ST1&0x01));

  // Read magnetometer data  
  uint8_t Mag[7];  
  I2Cread(MAG_ADDRESS,0x03,7,Mag);

  // Create 16 bits values from 8 bits data
  
  // Magnetometer
  int16_t mx = (Mag[1]<<8 | Mag[0]);
  int16_t my = (Mag[3]<<8 | Mag[2]);
  int16_t mz = (Mag[5]<<8 | Mag[4]);
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
  float declinationAngle = DECLINATION_ANGLE*DEG_2_RAD;
  heading += declinationAngle;

  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;

  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;

  // Convert radians to degrees for readability.
  float headingDegrees = heading * RAD_2_DEG; 

  // Serial Monitor Output
  Serial.println(headingDegrees);
//  Serial.print("\rHeading:\t");
//  Serial.print(heading);
//  Serial.print(" Radians   \t");
//  Serial.print(headingDegrees);
//  Serial.print(" Degrees   \t");
//  Serial.println ("\t");

//  Serial.print ("Magnetometer readings:"); 
//  Serial.print ("\tMx:");
//  Serial.print (mx); 
//  Serial.print ("\tMy:");
//  Serial.print (my);
//  Serial.print ("\tMz:");
//  Serial.print (mz);  
//  Serial.println ("\t");
  
  // End of line
  delay(10); 
}
