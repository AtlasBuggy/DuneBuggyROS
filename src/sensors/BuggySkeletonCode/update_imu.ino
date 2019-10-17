#include <Wire.h>
#include "libraries/Adafruit_Sensor.h"
#include "libraries/Adafruit_BNO055.cpp"
#include "libraries/utility/imumaths.h"

/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

double imu_vals[4];
int safe_imu = 0;

bool isSafe_IMU() {
  return safe_imu == 4;
}

//IMU runs off the Wire library, so info must be requested from 'slave' devices, thus interrupts may not work
//Also, this uses the sda/scl pins, 20/21 on the mega instead of serial. We may have to pinMode these pins with PULLUP modifiers as per the Wire library 
/*
volatile boolean inService = false;
void serialInterrupt_IMU() {
  if(inService) return;
  inService = true;
  
  interrupts();
  while(!Serial.available());
  update_imu();
  
  inService = false;
}
*/

void init_imu() {
  Serial.begin(115200);
//  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  /* Display the current temperature */
//  int8_t temp = bno.getTemp();
//  Serial.print("Current Temperature: ");
//  Serial.print(temp);
//  Serial.println(" C");
//  Serial.println("");

  bno.setExtCrystalUse(true);

//  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
  
}

// only call this internally
bool update_imu() {
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2

 
  safe_imu = 0;
  imu::Quaternion quat = bno.getQuat();
  imu_vals[safe_imu++] = quat.x();
  imu_vals[safe_imu++] = quat.y();
  imu_vals[safe_imu++] = quat.z();
  imu_vals[safe_imu++] = quat.w();

  /* Display the floating point data */
  /*
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  Serial.print("X: ");
  Serial.print(euler.x());
  Serial.print(" Y: ");
  Serial.print(euler.y());
  Serial.print(" Z: ");
  Serial.print(euler.z());
  Serial.print("\t\t");
  */

  
  /* Quaternion data */
//  imu::Quaternion quat = bno.getQuat();
  
  
  

  /* Display calibration status for each sensor. */
  /*
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("CALIBRATION: Sys=");
  Serial.print(system, DEC);
  Serial.print(" Gyro=");
  Serial.print(gyro, DEC);
  Serial.print(" Accel=");
  Serial.print(accel, DEC);
  Serial.print(" Mag=");
  Serial.println(mag, DEC);

  delay(BNO055_SAMPLERATE_DELAY_MS);
  */
  return true;
}

void write_imu_vals() {
  Serial.print(imu_vals[0], 4);
  Serial.print("\t");
  Serial.print(imu_vals[1], 4);
  Serial.print("\t");
  Serial.print(imu_vals[2], 4);
  Serial.print("\t");
  Serial.print(imu_vals[3], 4);
  Serial.print("\t");
}
