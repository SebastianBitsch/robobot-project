// File:          regbot_1_4_wheels.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/LED.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
//#include <webots/distance_sensor.h>
#include <webots/PositionSensor.hpp>
#include <webots/LED.hpp>
#include <webots/GPS.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <sys/time.h>
#include "main.h"
#include "dist_sensor.h"
#include "command.h"

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node

// global variable for access to robot features
Robot * robot = NULL;
//const int WheelCnt = 2;
Motor *wheels[WheelCnt] = {NULL, NULL};
PositionSensor * wheelEncoder[WheelCnt] = {NULL, NULL};
GPS * gps = NULL;
//const int DistSensorCnt = 2;
DistanceSensor *ps[DistSensorCnt] = {NULL, NULL};
//const int LEDCnt = 1;
LED * leds[LEDCnt];
//const int LightCnt = 8;
//LightSensor * lightsensor[LightCnt] = {NULL};
Accelerometer * accDev = NULL;
Gyro * gyroDev = NULL;

DistanceSensor *lineSensorRaw[LineSensorCnt] = {NULL};


int main(int argc, char **argv)
{
  // create the Robot instance.
  int loop = 0;
  float dtf, maxSampleTime = 0.0, minSampleTime = 100.0;
  robot = new Robot();
  printf("# Robot REGBOT_3 (1) created\n");
  // get the time step of the current world.
  int timeStep = (int) robot->getBasicTimeStep(); // in ms
  printf("Basic step time is %d ms\n", timeStep);
//   GPS * gps;
  char wheelsNames[WheelCnt][20] = {"POLOLU_D25_LEFT", "POLOLU_D25_RIGHT"};
  char encoderNames[WheelCnt][30] = {"POLOLU_D25_ENC_LEFT", "POLOLU_D25_ENC_RIGHT"};
  for (int i = 0; i < WheelCnt ; i++)
  {
    wheels[i] = robot->getMotor(wheelsNames[i]);
    wheelEncoder[i] = robot->getPositionSensor(encoderNames[i]);
    wheelEncoder[i]->enable(timeStep);
  }
  gps = robot->getGPS("gps");
  gps->enable(timeStep);
//   double speedRef = 2;
//   double speed[WheelCnt] = {speedRef, speedRef}; // [rad/s]
//   int speedSet = 0; // [rad/s]
  for (int i = 0; i < WheelCnt; i++)
  {
    wheels[i]->setPosition(INFINITY);
    wheels[i]->setVelocity(0.0);
  }
  printf("# dist sensor create\n");
//   DistanceSensor *ps[2];
  char psNames[2][20] = { "ds_1_right", "ds_2_forward" };
  const int IR_time_step = 64;
  for (int i = 0; i < DistSensorCnt; i++) 
  { // sharp sensor
    printf("# dist sensor %d create\n", i);
    ps[i] = robot->getDistanceSensor(psNames[i]);
    ps[i]->enable(IR_time_step); 
//     WbDeviceTag ds = wb_robot_get_device(psNames[i]);
//     wb_distance_sensor_enable(ds, IR_time_step);
  }
  calibrateIr();
  // IMU
  accDev = robot->getAccelerometer("imu_acc");
  accDev->enable(timeStep); 
  gyroDev = robot->getGyro("imu_gyro");
  gyroDev->enable(timeStep);
  // line sensor
  char lsNames[8][8] = {"ls0", "ls1", "ls2", "ls3", "ls4", "ls5", "ls6", "ls7"};
  for (int i = 0; i < LineSensorCnt; i++)
  {
    lineSensorRaw[i] = robot->getDistanceSensor(lsNames[i]);
  }
  // LED interface
  char ledNames[LEDCnt][20] = {"led1", "led2"};
  for (int i = 0; i < LEDCnt; i++)
  {
    leds[i] = robot->getLED(ledNames[i]);
  }
  printf("# Set LED 1 to off\n");
  leds[0]->set(0);
  //
  printf("# going to main loop\n");
  timeval t1, t2, dt;
  gettimeofday(&t1, NULL);
  // initialize for main loop
  mainInit();
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) 
  {
    const double * gpsPos;
    gpsPos = gps->getValues();
    if (true)
    { // REGBOT main loop
      bool isErr = loopTick();
      if (isErr)
        break;
    }
    // detect lost timing
    gettimeofday(&t2, NULL);
    timersub(&t2, &t1, &dt);
    dtf = dt.tv_sec * 1000.0 + dt.tv_usec/1000.0;
    if (dtf > maxSampleTime)
      maxSampleTime = dtf;
    if (dtf < minSampleTime)
      minSampleTime = dtf;
    t1 = t2;
    //
    if (loop % (1000/timeStep) == 0)
    { // every second
      printf("loop: %d simTime %g mission time %g (simTime %gms, max %gms, min %gms), gps=%.4g,%.4g,%.4g\n", 
             loop, robot->getTime(), missionTime,
             robot->getBasicTimeStep(),
             maxSampleTime, minSampleTime,
             gpsPos[0], gpsPos[1], gpsPos[2]);
      maxSampleTime = 0.0;
      minSampleTime = 1000;
    }
    loop++;
  };
  // Enter here exit cleanup code.
  printf("Webot simulation of REGBOT terminated\n");
  delete robot;
  return 0;
}
