// MPU-9150 Accelerometer + Gyro + Compass + Temperature
// -----------------------------
//
// By arduino.cc user "frtrobotik" (Tobias Hübner)
//
//
// July 2013
//      first version
//
// Open Source / Public Domain
//
// Using Arduino 1.0.1
// It will not work with an older version,
// since Wire.endTransmission() uses a parameter
// to hold or release the I2C bus.
//
// Documentation:
// - The InvenSense documents:
//   - "MPU-9150 Product Specification Revision 4.0",
//     PS-MPU-9150A.pdf
//   - "MPU-9150 Register Map and Descriptions Revision 4.0",
//     RM-MPU-9150A-00.pdf
//   - "MPU-9150 9-Axis Evaluation Board User Guide"
//     AN-MPU-9150EVB-00.pdf
//
// The accuracy is 16-bits.
//
// Some parts are copied by the MPU-6050 Playground page.
// playground.arduino.cc/Main/MPU-6050
// There are more Registervalues. Here are only the most
// nessecary ones to get started with this sensor.

#include "main.h"
#include "mpu9150.h"
#include "eeconfig.h"
// I2C address depends on your wiring. Set i mpu9150.h file.
//static int MPU9150_I2C_ADDRESS = ADDRESS_MPU;


//Variables where our values can be stored
// int cmps[3] = {0,0,0};
// int accl[3] = {0,0,0};
// int gyro[3] = {0,0,0};
int gyrotemp = 8;
//float pitch;

// float cmps_x() { return cmps[0]; }
// float cmps_y() { return cmps[1]; }
// float cmps_z() { return cmps[2]; }
// int gyro_x() { return gyro[0]; }
// int gyro_y() { return gyro[1]; }
// int gyro_z() { return gyro[2]; }
// float accel_x() { return           accl[0] * 9.82 / 16384.0; }
// float accel_y() { return           accl[1] * 9.82 / 16384.0; }
// float accel_z() { return ( (float) accl[2] * 9.82 / 16384.0); }
//float tempGyro() { return ( (float)gyrotemp - 12412.0 ) / 340.0; }
// float getPitch() { return pitch; }

// data from imu
int16_t imuAcc[3]  = {0,0,0};
int16_t imuGyro[3] = {0,0,0};
int32_t offsetGyro[3] = {0,0,0};
int16_t imuMag[3]     = {0,0,0};
int16_t imuTemp;
bool    gyroOffsetDone = false;

uint32_t gyroOffsetStartCnt = 1000;

const int gyroSensitivity = 2; // 0 = 250 deg/sec, 1=500, 2=1000 3 = 2000 deg/sec
const float gyroScaleFac = 250.0 / 32768.0 * float(1 << gyroSensitivity);
int32_t gyroSteadyMaxValue = int(10.0 / gyroScaleFac); // in degrees/s
const int accSensitivity = 1;
const float accScaleFac = 2*9.82 / 32768.0 * float(1 << accSensitivity);

// int16_t MPU9150_readSensor(int addr);
// void MPU9150_setupCompass();
// int MPU9150_writeSensor(int addr,int data);


/**
 * write one byte to a sensor (i2c) register
 * \param addr is i2c 7-bit address
 * \param reg is (8-bit) register to write to
 * \param data is 8-bit data to write
 * \returns 0 in succes and 1, 2, 3, 4 on error */
// int writeSensor(uint8_t i2caddr, int reg, int data)
// {
//   return 0;
// }

/**
 * Request a number of data from an i2c client
 * the data is in the Wire (or Wire1) buffer and read by Wire.read() called
 * \param i2caddr is the 7-bit address of the device ox68 for imu or 0x0c for readMagnetometer
 * \param reg is register number to read from
 * \param readCnt is number of registers to read
 * \returns number of characters available in read buffer */
// int readToBuffer(uint8_t i2cAddr, uint8_t reg, uint8_t readCnt)
// { //
//   int a = 0;
//   //
//   return a;
// }


// bool MPU9150_init()
// {
//   int err = 0;
//   return err == 0;
// }


//http://pansenti.wordpress.com/2013/03/26/pansentis-invensense-mpu-9150-software-for-arduino-is-now-on-github/
//Thank you to pansenti for setup code.
//I will documented this one later.
// void MPU9150_setupCompass(){
// }
// 
// 
// int16_t MPU9150_readSensor(int addr){
//   return 0;
// }
// 
// int MPU9150_writeSensor(int addr,int data){
//   return 0;
// }


/**
 * Request data starting from register 0x3B
 * Start a receice cycle to get 14 bytes of data
 * The data is available in rx buffer when Wire.done returns 1 (or Wire.finish() returns 1) */

// bool mpuRequestData()
// { // request data from IMU (ACC and Gyro - starting at register 0x3B
//   uint8_t e = 0;
//   return e == 0;
// }
// 
// /**
//  * finish current RX operation (or timeout after 200us) */
// bool mpuDataReady()
// {
//   return true;
// }

/**
 * Read 2 bytes from receive buffer and 
 * \return as 16 bit value */
// inline int16_t readInt16()
// {
//   return 0;
// }

/**
 * Read IMU data - assumed to be called some time after the mpuRequestData()
 * utilizing the time for the data to be received for other purposes.
 * If the data is not ready, then this function blocks until
 * all data is received or timeout (500us).
 * \returns true if all 14 bytes are read */
// int mpuReadData()
// {
//   int b = 0;
//   return b;
// }

////////////////////////////////////////////////

void eePromSaveGyroZero()
{
  eeConfig.push32(offsetGyro[0]);
  eeConfig.push32(offsetGyro[1]);
  eeConfig.push32(offsetGyro[2]);
}

void eePromLoadGyroZero()
{
  int skipCount = 4*3;
  if (not eeConfig.forDemoRunOnly() and robotId > 0)
  {
    offsetGyro[0] = eeConfig.read32();
    offsetGyro[1] = eeConfig.read32();
    offsetGyro[2] = eeConfig.read32();
    gyroOffsetDone = true;
  }
  else
  { // just skip, leaving default settings
    eeConfig.skipAddr(skipCount);
  }
}

//////////////////////////////////////////////////////////

// void readMagnetometer()
// { // must be read by the MPU9150 first, by setting up a set of registers
//   // and then reading the buffer registers in MPU9150
//   // - some may be set up in initializeer of MPU9150
// }

////////////////////////////////////////////

void sendStatusMag()
{
  const int MRL = 250;
  char reply[MRL];
  snprintf(reply, MRL, "mag %d %d %d\r\n",
           imuMag[0], imuMag[1], imuMag[2]);
  usb_send_str(reply);
}
