/***************************************************************************
 *   Copyright (C) 2016 by DTU                             *
 *   jca@elektro.dtu.dk                                                    *
 *
 *   Main function for small regulation control object (regbot)
 *   build on a small 72MHz ARM processor MK20DX256,
 *   intended for 31300 Linear control
 *   has an IMU and a dual motor controller with current feedback.
 *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************//*
  This file contains all the functions used for calculating
  the frequency, real Power, Vrms and Irms.
*/
#include "motor_controller.h"
#include <math.h>
#include <stdlib.h>
#include "robot.h"
#include "main.h"
#include "control.h"
#include "eeconfig.h"
#include "pins.h"

#define MOTOR1   0
#define MOTOR2   1

// Variables
static volatile int   rotations[2] = {0};
float motorAnkerVoltage[2];
// float max_motor_voltage = 19;
int16_t motorAnkerPWM[2] = {0,0};
int8_t  motorEnable[2] = {0,0};
bool motorPreEnabled = true;
bool motorPreEnabledRestart = true;
/**
 * if motor gets full power in more than 1 second, then stop */
int overloadCount = 0;

/// encoder
// encoder-d value is in radians on wheel axle
// translated into encoder in robot.cpp
double encoderd[2];
double encoderdLast[2] = {0,0};
double encStartTime[2] = {0.0};

uint32_t encoder[2];
uint32_t encoderLast[2] = {0,0};
uint32_t  encPeriod_cpu[2];
bool     encCCV[2];
bool     encTimeOverload_cpu[2];

// encoder calibration varables
uint32_t encStartTime_cpu[2]; /// last encoder tick CPU count (used during collect data for calibration)
uint32_t encTime_cpu[2][MAX_ENC_VALS * 2]; /// collected timing data (2 revolutions)
uint32_t encTimeScale[2][MAX_ENC_VALS]; /// calibration values * 1000
bool encTimeScaleCalibrated = false; /// calibrated values are usable (if use flag is true)
bool encTimeTryCalibrate = true; /// try calibrate or retry calibrate when speed is right
bool encTimeStopCollect = false; /// save a bit of time and values used for calibration
bool encTimeCalibrateUse = true; /// ability to not use a calibrated system, even if calibrated
bool encTimeTryReindex = false;  /// used after loading configuration from flash
int encReindexBestIndex = -1;

// test better encoder value end

float motorCurrentA[2];  // in amps
uint16_t motorCurrentM[2]; // external current sensor
int32_t motorCurrentMOffset[2];
uint8_t pinMotor2Dir = M2DIR1;
uint8_t pinMotor2Pwm = M2PWM1;

/**
 * Read motor current
 * \param motor 0 or 1 for motor 0 or 1.
 * \returns motor current in amps */

void motorInit(void)
{
}


void motorSetAnchorVoltage()
{
  float batteryNominalVoltage = batVoltInt * batVoltIntToFloat;
  if (batteryNominalVoltage < 5.0)
    // not on battery - justfor test and avoid divide by zero
    batteryNominalVoltage = 11.1;
  const float max_pwm = 4096;
  float scaleFactor = max_pwm / batteryNominalVoltage;
  // limit motor voltage
//   // left motor
//   if (motorAnkerVoltage[0] > max_motor_voltage)
//   {
//     motorAnkerVoltage[0] = max_motor_voltage;
//     overloadCount++;
//   }
//   else if (motorAnkerVoltage[0] < -max_motor_voltage)
//   {
//     motorAnkerVoltage[0] = -max_motor_voltage;
//     overloadCount++;
//   }
//   else
//     overloadCount = 0;
//   // right motor
//   if (motorAnkerVoltage[1] > max_motor_voltage)
//   {
//     motorAnkerVoltage[1] = max_motor_voltage;
//     overloadCount++;
//   }
//   else if (motorAnkerVoltage[1] < -max_motor_voltage)
//   {
//     motorAnkerVoltage[1] = -max_motor_voltage;
//     overloadCount++;
//   }
//   else
//     overloadCount = 0;
  // overload check
  if (overloadCount > 500)
  { // disable motor (after 0.5 second)
    motorSetEnable(0, 0);
  }
  // debug
  if (true)
  { // log en extra logger set as item 1
    dataloggerExtra[0] = overloadCount;
  }
  // debug end
  // convert to PWM values (at 20khz)
  int w1, w2;
  w1 = int16_t(motorAnkerVoltage[0] * scaleFactor);
  // the right motor must run the other way
  w2 = int16_t(-motorAnkerVoltage[1] * scaleFactor);
  // implement
  motorSetAnkerPWM(w1, w2);
#ifdef WEBOT
  // Kemf * 590 RPM@6V@250mA@Ra=3ohm jf. Pololu 9.7:1 Metal Gearmotor 25Dx48L mm LP 6V
  //         590 * 2 * pi
  // Kemf * ------------- = 6 - 3*0.25
  //              60
  // motor regnet med gear (9.68), ellers 5700 RPM
  // Kemf = 0.085 [V s]
  const double Kemf = 0.085; 
  const double rA = 3; // Ohm
  const double VrollFriction = 1.0;
  double torque;
  if (motorEnable[0])
  {
    const double Vemf = wheels[0]->getVelocity() * Kemf;
    double va = motorAnkerVoltage[0] - Vemf;
    motorCurrentA[0] = va/rA;
    motorCurrentMLowPass[0] = setMotorCurrentM(0, motorCurrentA[0]);
    // implement roll friction
    // corresponding to 1v on motor will not run the motor
    if (fabs(va) < VrollFriction)
      va = 0.0;
    else if (va > 0)
      va = va - VrollFriction;
    else
      va = va + VrollFriction;
    // friction in motor webots model
    // B = 0.0015 eller 0.001
    torque = va/rA * Kemf;
    wheels[0]->setTorque(torque); // left
  }
  if (motorEnable[1])
  {
    const double Vemf = wheels[1]->getVelocity() * Kemf;
    double va = motorAnkerVoltage[1] - Vemf;
    motorCurrentA[1] = va/rA;
    motorCurrentMLowPass[1] = setMotorCurrentM(0, motorCurrentA[1]);
    // implement roll friction
    // corresponding to 1v on motor will not run the motor
    if (fabs(va) < VrollFriction)
      va = 0.0;
    else if (va > 0)
      va = va - VrollFriction;
    else
      va = va + VrollFriction;
    wheels[1]->setTorque(va/rA * Kemf); // right
  }
#endif
}

/**
 * e2 used on hardware < 3 only */
void motorSetEnable(uint8_t e1, uint8_t e2)
{
  if (motorPreEnabled and (e1 or e2) and not (motorEnable[0] or motorEnable[1]))
  { // switch off current zero offset calculation
//     const char MSL = 100;
//     char s[MSL];
    motorPreEnabled = false;
    motorPreEnabledRestart = true;
    // not needed logIntervalChanged();
//     snprintf(s, MSL, "# current A/D*300 offset  %ld %ld fac %d raw %d %d.\r\n", 
//              motorCurrentMOffset[0], motorCurrentMOffset[1], 
//              lowPassFactor,
//              motorCurrentM[0], motorCurrentM[1]
//             );
//     usb_send_str(s);
  }
  // reset overload
  if (e1 and not motorEnable[0])
    overloadCount = 0;
  // enable motors (or disable)
  motorEnable[0] = e1;
  motorEnable[1] = e2;
  if (not motorEnable[1])
    wheels[0]->setTorque(0);
  if (not motorEnable[1])
    wheels[1]->setTorque(0); // right
}

/** 
 * allowed input is +/- 2048, where 2048 is full battery voltage
 * */
void motorSetAnkerPWM(int m1PWM, int m2PWM)
{ // too small PWM will not implement
}


/////////////////////////////////////////////

uint32_t encoderTimeAverage(int motoridx)
{
  uint32_t avg = 0;
  for (int i = 0; i < MAX_ENC_VALS; i++)
    avg += encTime_cpu[motoridx][i];
  avg /= MAX_ENC_VALS;
  return avg;
}

///////////////////////////////////////////////

/** velocity history for stability test */
float velHist[2] = {0,0};
int velHistWait = 100;


bool calibrateEncoderTest()
{
  velHist[0] = velHist[0] * 0.95 + wheelVelocityEst[0] * 0.05;
  velHist[1] = velHist[1] * 0.95 + wheelVelocityEst[1] * 0.05;
  //
  if (velHistWait == 0)
  {
    if (fabsf(wheelPosition[0]) > 0.05 and
        fabsf(wheelPosition[1]) > 0.05 and 
      fabsf(wheelVelocityEst[0]) > 0.2 and
      fabsf(wheelVelocityEst[1]) > 0.2 and
      not balance_active and
      fabsf(velHist[0] - wheelVelocityEst[0]) < 0.1 and
      fabsf(velHist[1] - wheelVelocityEst[1]) < 0.1
      )
    { // velocity high and stable
      bool isOK = true;
      for (int m = 0; m < 2; m++)
      {
        uint32_t vsum = 0;  
        uint32_t difSum = 0;
        for (int i = 0; i < MAX_ENC_VALS; i++)
        {
          uint32_t ti = encTime_cpu[m][i];
          vsum += ti;
          difSum += abs(int(ti) - int(encTime_cpu[m][i + MAX_ENC_VALS]));
        }
        isOK = difSum < (vsum / (5 * MAX_ENC_VALS));
        const int MSL = 100;
        char s[MSL];
        if (false)
        {
          snprintf(s, MSL, "#enc test ok=%d, vsum=%u, difsum=%u\n", 
                  isOK, vsum, difSum);
          usb_send_str(s);
        }
        if (not isOK)
          break;
      }
      if (isOK)
      {
        if (encTimeTryReindex)
          calibrateEncoderIndex();
        else
          calibrateEncoder();
        usb_send_str("# motor_controller::calibrateEncoderTest: calibrate encoder succeeded\n");
      }
//       else
//         usb_send_str("# motor_controller::calibrateEncoderTest: failed\n");
    }
    // wait for new values
    velHistWait = 20;
  }
  else
    velHistWait--;
  return encTimeScaleCalibrated;
}


void calibrateEncoder()
{ // assuming velocity is constant and encoder is running forward at constant velocity
  // - calculate average across 2 x 48 timing measurements (two revolutons)
  // - calculate factor (* 1000) for each cell to reach this average, i.e.
  // - normalized over the average over the dataset (2x48 measurements)
  // stop collecting data
  encTimeStopCollect = true;
  // calculate compensation factors
  for (int m = 0; m < 2; m++)
  {
    uint32_t avg = encoderTimeAverage(m);
    for (int i = 0; i < MAX_ENC_VALS; i++)
    { // factor values are scaled a factor 1000 (over 2x48 periods, so 2000)
      encTimeScale[m][i] = (avg * 2000) /
             (encTime_cpu[m][i] + encTime_cpu[m][i + MAX_ENC_VALS]);
    }
  }
  encTimeScaleCalibrated = true;
  // restart collecting data
  encTimeStopCollect = false;
}

////////////////////////////////////////////////////


bool calibrateEncoderIndex()
{
  bool result = false;
  return result;
}

void eePromSaveEncoderCalibrateInfo()
{
  char v = 0x00;
  if (encTimeTryCalibrate)
    v |= 0x01;
  if (encTimeCalibrateUse)
    v |= 0x02;
  if (encTimeScaleCalibrated)
    v |= 0x04;
  eeConfig.pushByte(v);
  if (encTimeScaleCalibrated)
  { // save number of factors
    eeConfig.pushByte(MAX_ENC_VALS);
    // save all factors
    for (int m = 0; m < 2; m++)
    { // both motors
      for (int i = 0; i < MAX_ENC_VALS; i++)
      {
        eeConfig.pushWord(uint16_t(encTimeScale[m][i]));
      }
    }
  }
}


void eePromLoadEncoderCalibrateInfo()
{
  char v = eeConfig.readByte();
//  char v = eeprom_read_byte((uint8_t*)eePushAdr++);
  bool calibrated = (v & 0x04) == 0x04;
  // number of bytes to skip if not robot-specific configuration
  int skipCount = 0;
  int encTickCnt = 0;
  if (calibrated)
  {
    encTickCnt = eeConfig.readByte();
    // calibration values are not relevant if string configuration
    // 2 motors, 2 byte each value
    skipCount = 2 * encTickCnt * 2;
  }
  if (not eeConfig.forDemoRunOnly())
  { // load from flash
    encTimeTryCalibrate = (v & 0x01) == 0x01;
    // NB!
    // do not use calibrated values at boot, as it is worse than nothing when not in sync with encoder
    encTimeCalibrateUse = false; // (v & 0x02) == 0x02;
    //
    encTimeTryReindex = calibrated;
    encTimeScaleCalibrated = calibrated;
    if (calibrated)
    { // read calibration values for both motors
      for (int m = 0; m < 2; m++)
      { // both motors
        for (int i = 0; i < encTickCnt; i++)
        { // read all calibration values
          uint16_t v = eeConfig.readWord();
          if (i < MAX_ENC_VALS)
            // do not write outside array - if size has changed
            encTimeScale[m][i] = v;
          else
          { // data size changet, not usable values
            encTimeTryReindex = false;
          }
        }
      }
    }
  }
  else
    // load from hard-coded mission
    eeConfig.skipAddr(skipCount);
}
