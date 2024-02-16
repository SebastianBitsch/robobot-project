 /***************************************************************************
 * definition file for the regbot.
 * The main function is the controlTick, that is called 
 * at a constant time interval (of probably 1 ms, see the intervalT variable)
 * 
 * The main control functions are in control.cpp file
 * 
 *   Copyright (C) 2017 by DTU                             *
 *   jca@elektro.dtu.dk                                                    *
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
 ***************************************************************************/

#include "control.h"
#include "data_logger.h"
#include "motor_controller.h"
//#include "serial_com.h" 
#include "robot.h"
#include "mission.h"
#include "linesensor.h"
#include "dist_sensor.h"
#include "eeconfig.h"
#include "servo.h"
#include "mpu9150.h"

// mission state 0 is waiting for start button
// mission state 999 is end - finished
//int16_t missionState;
int16_t missionStateLast;
int8_t sendStatusState = 0;
float mission_turn_ref = 0.0;    /// reference angle for heading control
float mission_vel_ref = 0.0;     /// base velocity
float mission_wall_vel_ref = 0.0;     /// added velocity from IR sensor controll (wall follow in velocity mode)
//float mission_vel_ref_preturn = 0.0;     /// base velocity
float mission_vel_ref_ref = 0.0; /// base velocity reference to velocity control (ss only)
float mission_pos_ref = 0.0;     /// base velocity (ss only)
bool mission_pos_use = false;    // should position regulator be used
float mission_line_ref = 0.0;    /// where should line edge be - when line following [cm]
bool  mission_line_LeftEdge = true;
float mission_wall_ref = 0.2;    /// what distance - when wall following [m]
//bool  mission_wall_sensor1 = true; /// first sensor 1=side 2=fwd
bool mission_wall_turn = true;
//bool mission_wall_sign = false;
int8_t mission_irSensor_use = false;
bool mission_turn_do = false;    /// is in a turn - specified by turn radius
float mission_turn_angle = 0;       /// this is the turn angle during a turn-radius turn
float mission_turn_radius = 0.0; /// turn radius when doing a turn
float regul_bal_uvel = 0.0; /// output of balance regulator
bool backToNormal = false; // revert to flash default after hard mission
/**
  * Parameters for balance control */
bool balance_active = 0;    /// Try to use balance control
/** remote control variables */
bool remoteControl = false;
float remoteControlVel = 0.0; /// in m/s
float remoteTurnrate = 0.0; /// in m/s
bool remoteControlNewValue = false;
// /** line follow regulator */
bool regul_line_use = false;  // use heading control from line sensor
//bool regul_line_followLeft = true;
//

// int mission = 0;
int missionButtonCount = 0;
const int missionMax =4;
const char * missionName[missionMax] = // NB! NO spaces in name"
                            {"User_mission",
                              "Balance_on_spot",
                              "Balance_square",
                              "Follow_wall"
                            };
UMissionLine * misLine = NULL;  // current mission line    "User_mission"};
UMissionLine * misLineLast = NULL;  // current mission line    "User_mission"};
uint8_t misThread = 0;  // current thread number (setwhen mis-line is set)
// uint8_t misThreadLast = 0;  // last thread number (setwhen mis-line is set)

int8_t missionLineNum = 0;         // current mission line number
//int misLineNumLast = 0;
//int misStartTime;           // start time for this line
// float misLastAng;          // start heading for this line
// float misAngSum;            // turned angle for this line
float misStartDist;         // start distance for this line
int misLast = -1;
int misStateLast = -1;
  

//////////////////////////////////////  
//////////////////////////////////////  
//////////////////////////////////////  

/** constructor */
UControl::UControl()
{ // create regulators on heap
  ctrlCurLeft = new UControlBase("cmcu");
  ctrlCurRight = new UControlBase("cmcu"); // same ID (always equal parameters)
  ctrlVel = new UControlBase("cvel");
    ctrlTurnrate = new UControlBase("ctra"); 
  ctrlTurn = new UControlTurn("ctrn");
  ctrlWallVel = new UControlIrVel("cwve");
  ctrlWallTurn  = new UControlWallTurn("cwth");
  ctrlPos  = new UControlPos("cpos");
  ctrlEdge  = new UControlEdge("cedg");
  ctrlBal = new UControlBase("cbal");
  ctrlBalVel = new UControlBalVel("cbav");
  ctrlBalPos = new UControlPos("cbap");
  // set mission flags
  controlActive = false;
  missionStateLast = -1;
  mission = 0;
  missionState = 0;
  rateLimitUse = false;
  rateLimitUsed = false;
  // connect regulator in and out
  setRegulatorInOut();
  tick = 0;
}

void UControl::setRemoteControl(bool enable, float vel, float velDif, bool balance)
{
  if (remoteControl and not enable and missionState > 0)
  { // turning off remote control - need to reset some parts
    mission_turn_ref = pose[2];
    // acceleration limitation must start from current velocity
        ref_vel_after_acc_limit = robotVelocity;
    balance_active = savedBalanceActive;
    controlActive = savedControlActive;
  }
  else if (enable and not remoteControl)
  { // turn on remote control
    savedBalanceActive = controlActive and balance_active;
    savedControlActive = controlActive;
    // start servo steering
    // servo.servo1isSteering = true;
//     servo.setServoSteering();
    // make sure control is active
    controlActive = true;
    motorSetEnable(true, true);
//     usb_send_str("# UControl::setRemoteControl to RC\n");
  }
  remoteControl = enable;
  remoteControlVel = vel;
    remoteTurnrate = velDif;
  remoteControlNewValue = true;
  balance_active = balance;
}

void UControl::sendMissionStatusChanged(bool forced)
{
  const int MSL = 120;
  char s[MSL];
  if (forced or mission != misLast or missionState != misStateLast or 
     (misLine != NULL and (misLine != misLineLast or misThread != misThreadLast)))
  {
    // mission thread and line number is of the latest implemented line and thread
    snprintf(s, MSL, "mis %d %d %d '%s' %d %d\r\n",
            mission, missionState , missionLineNum, 
            missionName[mission],
            0, // unused (was sendStatusWhileRunning)
            misThread
          );
    usb_send_str(s);
    misLast = mission;
    misStateLast = missionState;
    misLineLast = misLine;
    misThreadLast = misThread;
  }
  if (false and misLine != NULL and (misLine != misLineLast or misThread != misThreadLast))
  {
    char * p1;
    int n;
    snprintf(s, MSL, "#mis %d %d %d %d'",
             mission, missionState, missionLineNum, misThread);
    n = strlen(s);
    p1 = &s[n];
    n += misLine->toString(p1, MSL - n - 4, false);
    p1 = &s[n];
    strncpy(p1, "'\r\n", 4);
    usb_send_str(s);
    misLineLast = misLine;
    misThreadLast = misThread;
  }
}

/**
 * Step on position, as specified by GUI
 * parameter estimate mission (should use fast logging) */
bool UControl::mission_hard_coded()
{
  const int MSL = 60;
  char s[MSL];
  bool isOK = false;
  switch (missionState)
  {
    case 0: // wait for start button
      if (button or missionStart)
      { // goto next state
        missionTime = 0.0;
        missionState = 1;
        missionStart = false;
//         controlActive = false;
        snprintf(s, MSL, "# loading hard coded mission %d (idx=%d)\r\n", mission, mission - 1);
        usb_send_str(s);
      }
      break;
    case 1:
      if (missionTime > 0.5) // wait for finger away from button
      { // go to next state
        bool isOK;
        bool back;
        missionTime = 0.0;
        missionState = 0;
        back = mission > 0;
        snprintf(s, MSL, "# loading hard coded mission %d (idx=%d) now.\r\n", mission, mission - 1);
        isOK = eeConfig.hardConfigLoad(mission - 1, false);
        snprintf(s, MSL, "# implement as mission %d (OK=%d)\r\n", mission, isOK);
        usb_send_str(s);
        // start the newly loaded mission
        if (isOK)
        {
          missionStop = false;
          missionStart = true;
          backToNormal = back;
        }
      }
      break;
    default:
      break;
  }
  return isOK;
}

///////////////////////////////////////////////////////////////////////


/**
 * run mission from user lines */
void UControl::user_mission()
{
  bool irP = false, lsP = false, chirpExtraLog = false;
  switch (missionState)
  {
    case 0: // wait for start button
      if (button or missionStart)
      { // goto next state
        missionTime = 0.0;
        missionStop = false;
        // chirp must be reset (used or not)
        chirpReset(logInterval);
        // log interval is number of values each 360 deg of chirp (3-5 or so).
        //         controlActive = false;
//         if (button)
        {
          missionState = 6;
          // tell bridge (and others) that a green button is pressed
          usb_send_str("event 33\n");
          usb_send_str("# send event 33 (start mission)\n");
        }
//         else
//           missionState = 2;
        userMission.getPowerUse(&lsP, &irP, &chirpExtraLog);
        if (irP)
        { // takes time to get first value
          // usb_send_str("# IR power on\n");
          setIRpower(true);
        }
        if (lsP)
        { // turns on when used - that is enough
          lineSensorOn = true;
          lsEdgeValidCnt = 0;
          crossingLineCnt = 0;
          // usb_send_str("# line sensor power on\n");
        }
        if (chirpExtraLog)
        {
          usb_send_str("# chirp found\n");
        }
        // enable or disable chirp log
        logRowFlags[LOG_CHIRP] = chirpExtraLog;
        // reset log buffer (but do not start logging)
        initLogStructure(50);
        // clear extra logger data (may not be used)
        memset(dataloggerExtra, 0, sizeof(dataloggerExtra));
        // first quit second should be quiet
        // after a RC session, motors are enabled
        motorSetEnable(false, false);
        // send start event to client
        // this can be used as delayed start
//         usb_send_str("event 33\n");
      }
      break;
    case 1: // wait to allow release of button
//       usb_send_str("#user_mission:: state 1\n");
//       if (time > 1.0)
      { // init next state
        if (userMission.startMission())
        {
          missionState = 2;
          // forget all old values
          resetControl();
          // make control active (if not already)
          controlActive = true;
//           motorSetEnable(true, true);
          //usb_send_str("# user_mission -> state 2\n");
        }
        else
        {  // no user defined lines
          missionState = 9;
          break;
        }
//         userMission.getPowerUse(&lsP, &irP);
//         if (irP)
//         { // takes time to get first value
//           // usb_send_str("# IR power on\n");
//           setIRpower(true);
//         }
//         if (lsP)
//         { // turns on when used - that is enough
//           lineSensorOn = true;
//           // usb_send_str("# line sensor power on\n");
//         }
//         motorSetEnable(true, true);
      }
      break;
    case 2:
      // run user mission lines
      { // test finished, and move to next line if not finished
        bool theEnd = userMission.testFinished();
        if (theEnd or missionStop or button)
        { // finished - mission end
          // debug
          const int MSL = 50;
          char s[MSL];
          snprintf(s, MSL, "# stop vel=%g m/s, but=%d, ctrlVel=%g\n", mission_vel_ref, button, ref_vel);
          usb_send_str(s);
          snprintf(s, MSL, "# ir_use=%d, wall_turn=%d, pos=%d\n", mission_irSensor_use , mission_wall_turn, mission_pos_use);
          usb_send_str(s);
          // debug end
          missionState=8;
          endTime = missionTime + 0.05;
        }
      }
      break;
    case 6:
//       usb_send_str("#user_mission:: state 6\n");
      
      if (missionTime > 0.1 and not button)
        missionState = 7;
      else if (missionTime > 1.2)
      {
        missionState = 1;
        if (missionButtonCount > 0)
        {
          mission = missionButtonCount;
          // debug
          const char MSL = 50;
          char s[MSL];
          snprintf(s, MSL, "# (6) starting mission %d\n", missionButtonCount);
          usb_send_str(s);
        }
      }
      break;
    case 7:
//       usb_send_str("#user_mission:: state 7\n");
      if (button)
      {
        missionState = 6;
        missionButtonCount ++;
        usb_send_str("# button pressed again!\n");
      }
      else if (missionTime > 1.2)
      {
        missionState = 1;
        if (missionButtonCount > 0)
        {
          mission = missionButtonCount;
          // debug
          const char MSL = 50;
          char s[MSL];
          snprintf(s, MSL, "# (7) starting mission %d\n", missionButtonCount);
          usb_send_str(s);
        }
      }
      break;
    case 8:
      // stop (start) button pressed - mission ended
//       usb_send_str("# user_mission (state 8)\n");
      // stop movement, and
      mission_vel_ref = 0;
      // wait for start button to be released (or 0.5 second)
      if (not button and missionTime > endTime)
      { // to default state - stop
//         usb_send_str("# user_mission (8) -> the end!\n");
        missionState = 9;
        userMission.stopMission();
      }
      break;
    default:
//       usb_send_str("# the end! (mission stop)\n");
      missionStop = true;
      usb_send_str("event 0\n");
      break;
  }
}



void UControl::eePromSaveCtrl()
{ // must be called in in right order
  eeConfig.pushByte(mission);
  ctrlCurLeft->eePromSave();
  ctrlCurRight->eePromSave();
  ctrlVel->eePromSave();
    ctrlTurnrate->eePromSave();
  ctrlTurn->eePromSave();
  ctrlWallVel->eePromSave();
  ctrlWallTurn->eePromSave();
  ctrlPos->eePromSave();
  ctrlEdge->eePromSave();
  ctrlBal->eePromSave();
  ctrlBalVel->eePromSave();
  ctrlBalPos->eePromSave();
}

void UControl::eePromLoadCtrl()
{ // must be called in in right order
  mission = eeConfig.readByte();
  if (eeConfig.isStringConfig())
    // anny mission must run as mission 0
    mission = 0;
//   usb_send_str("# loaded mission type\n");
  ctrlCurLeft->eePromLoad();
  ctrlCurRight->eePromLoad();
  ctrlVel->eePromLoad();
    ctrlTurnrate->eePromLoad();
  ctrlTurn->eePromLoad();
  ctrlWallVel->eePromLoad();
  ctrlWallTurn->eePromLoad();
  ctrlPos->eePromLoad();
  ctrlEdge->eePromLoad();
  ctrlBal->eePromLoad();
  ctrlBalVel->eePromLoad();
  ctrlBalPos->eePromLoad();
}


void UControl::setRegulatorInOut()
{ // set initial input and output for regulators
  // turn controllers
  ctrlWallVel->setInputOutput(&mission_wall_vel_ref, irDistance, &ref_vel);
  ctrlWallTurn->setInputOutput(&mission_wall_ref, irDistance, &ref_turnrate, 0.3);
  // left and right line valid is now one value only
  ctrlEdge->setInputOutput(&mission_line_ref, &lsLeftSide, &lsRightSide, &lsEdgeValid/*, &lseRightValid*/, 
                           &ref_turnrate, &mission_line_LeftEdge);
  ctrlTurn->setInputOutput(&ref_turn_angle /* &mission_turn_ref*/, &pose[2], &ref_turnrate);
  // position controller
  ctrlPos->setInputOutput(&mission_pos_ref, &distance, &misStartDist, &ref_vel);
  // balance controllers
  ctrlBalPos->setInputOutput(&mission_pos_ref, &distance, &misStartDist, &ref_vel);
  ctrlBalVel->setInputOutput(&ref_vel_after_acc_limit, &robotVelocity, &ref_balance_tilt);
  ctrlBal->setInputOutput(&ref_balance_tilt, &pose[3], &u_balance, &gyroTiltRate);
  // wheel velocity and delta velocity control
  ctrlVel->setInputOutput(&ref_vel_after_acc_limit, &robotVelocity, &u_vel);
    ctrlTurnrate->setInputOutput(&ref_turnrate, &robot_delta_velocity, &u_turnrate);
  // motor current control
  ctrlCurLeft->setInputOutput(&ref_cur[0], &motorCurrentA[0], &motorAnkerVoltage[0]);
  ctrlCurRight->setInputOutput(&ref_cur[1], &motorCurrentA[1], &motorAnkerVoltage[1]);
}

bool UControl::setRegulator(const char* line)
{ // set parameters from string
  bool used;
//   usb_send_str("# 1 control\r\n");
  used = ctrlCurLeft->setRegulator(line);
  if (used)
    // set also regulator for other wheel with same values
    used = ctrlCurRight->setRegulator(line);
  if (not used)
    used = ctrlVel->setRegulator(line);
  if (not used)
    used = ctrlTurnrate->setRegulator(line);
  if (not used)
    used = ctrlTurn->setRegulator(line);
  if (not used)
    used = ctrlWallVel->setRegulator(line);
  if (not used)
    used = ctrlWallTurn->setRegulator(line);
  if (not used)
    used = ctrlPos->setRegulator(line);
  if (not used)
    used = ctrlEdge->setRegulator(line);
  if (not used)
    used = ctrlBal->setRegulator(line);
  if (not used)
    used = ctrlBalVel->setRegulator(line);
  if (not used)
    used = ctrlBalPos->setRegulator(line);
  return used;
}

bool UControl::sendStatusControl ( const char* line )
{ // line is request ID for the controller to send
  const int MSL = 270;
  char s[MSL];
  bool isOK = true;
  int n;
  if (ctrlBal->isMe(line))
    n = ctrlBal->getRegulator(s, MSL);
  else if (ctrlBalVel->isMe(line))
    n = ctrlBalVel->getRegulator(s, MSL);
  else if (ctrlBalPos->isMe(line))
    n = ctrlBalPos->getRegulator(s, MSL);
  else if (ctrlEdge->isMe(line))
    n = ctrlEdge->getRegulator(s, MSL);
  else if (ctrlPos->isMe(line))
    n = ctrlPos->getRegulator(s, MSL);
  else if (ctrlTurn->isMe(line))
    n = ctrlTurn->getRegulator(s, MSL);
  else if (ctrlVel->isMe(line))
    n = ctrlVel->getRegulator(s, MSL);
  else if (ctrlTurnrate->isMe(line))
    n = ctrlTurnrate->getRegulator(s, MSL);
  else if (ctrlCurLeft->isMe(line))
    n = ctrlCurLeft->getRegulator(s, MSL);
  else if (ctrlWallTurn->isMe(line))
    n = ctrlWallTurn->getRegulator(s, MSL);
  else if (ctrlWallVel->isMe(line))
    n = ctrlWallVel->getRegulator(s, MSL);
//   else if (strncmp(line, "ctrl", 4) == 0)
//     n = snprintf(s, MSL, "ctrl %d %g", rateLimitUse, rateLimit);
  else
  {
    isOK = false;
    n = 0;
  }
  if (isOK)
  {
    strncpy(&s[n], "\n", MSL - n);
    usb_send_str(s, true);
  }
  return isOK;
}


void UControl::resetControl()
{
  ctrlCurLeft->resetControl();
  ctrlCurRight->resetControl();
  ctrlVel->resetControl();
    ctrlTurnrate->resetControl();
  ctrlTurn->resetControl();
  ctrlWallVel->resetControl();
  ctrlWallTurn->resetControl();
  ctrlPos->resetControl();
  ctrlEdge->resetControl();
  ctrlBal->resetControl();
  ctrlBalVel->resetControl();
  ctrlBalPos->resetControl();
  ref_vel_after_acc_limit = 0;  
  ref_turnrate = 0;
  ref_cur[0] = 0;
  ref_cur[1] = 0;
  // and line sensor
  lsPostProcess();
}

void UControl::resetTurn()
{
    ref_turnrate = 0;
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

bool once = false;
bool oldRemoteControlOn = false;
bool remoteCtrlEnded = false;


void UControl::controlTick(void)  
{
//   const int MSL = 90;
//   char s[MSL];
  if (remoteControl)
  { // in remote control - possibly within a mission,
    // stop advancing mission
    if (not oldRemoteControlOn)
    {
      oldRemoteControlOn = true;
      usb_send_str("# UControl::controlTick: Shift to remote control\n");
    }
  }
  else if (mission > 0 and not backToNormal) 
    // may be start of hard coded mission
    mission_hard_coded();
  else
  { // current mission - or prepared hard coded mission
    user_mission();
  }
  if (not remoteControl and oldRemoteControlOn)
  {
    usb_send_str("# UControl::controlTick: Shift to auto (away from remote control)\n");
    if (controlActive)
      usb_send_str("# UControl::controlTick: control active\n");
    remoteCtrlEnded = true;
    oldRemoteControlOn = false;
  }
  //
  // now to the control valid in this tick period
  //
  if (controlActive)
  { // ready to do control loops
    // use turn ref as is in most cases
    ref_turn_angle = mission_turn_ref;
    if (chirpRun)
    { // we are using chirps, so calculate current addition to control value
      // amplitude limitation
      float dv = fabs(wheelVelocityEst[0] - mission_vel_ref);
      if (chirpDestination == 0 and dv > chirpVelMax)
      { // new max amplitude
        chirpVelMax = dv;
      }
      // phase change in sample time
      double dp = chirpFrq * SAMPLETIME;
      // new phase angle
      chirpAngle += dp;
      // fold
      if (chirpAngle > 2*M_PI)
      { // a full cycle
        chirpLogQuadrant = 1;
        chirpAngle  -= 2*M_PI;
      }
      if (chirpAngle > 2 * M_PI * chirpLogQuadrant / float(chirpLogInterval))
      { // passed a log quadrant (or 2 PI)
        chirpLogQuadrant++;
        // tell log system to log
        chirpLog = true;
        // dectrase frequency at every log sample
        chirpFrq *= chirpFrqRate;
        if (chirpDestination == 0)
        {
          if (chirpVelMax > 0.5 or chirpMotorVoltMax > 7.0)
          { // reduce chirp amplitude and reset amplitude
            chirpMotorVoltMax = 5.0;
            chirpVelMax = 0.3;
            chirpAmplitude *= 0.99;
          }
        }
      }
      chirpValue = chirpAmplitude * cos(float(chirpAngle));
      tick++;
    }
    else
      tick = 0;
    //
    if (remoteControl)
    { // velocity comes from remote
      ref_vel = remoteControlVel;
      ref_turnrate = remoteTurnrate;
    }
    else
    { // velocity comes from mission
      ref_vel = mission_vel_ref;
      if (chirpRun)
      { // modify velocity or turn reference
        if (chirpDestination == 0)
                    ref_vel += chirpValue;
        else if (chirpDestination == 1)
                    ref_turn_angle += chirpValue;
      }
      if (mission_irSensor_use and not mission_wall_turn)
      { // front distance is controlling velocity reference
        // balance or not
        ctrlWallVel->outLimitUsed = rateLimitUsed;
        ctrlWallVel->controlTick();
        // further away should increase speed
        ref_vel *= -1.0;
        if (ref_vel > mission_vel_ref)
          ref_vel = mission_vel_ref;
        else if (regul_line_use and ref_vel < 0)
          ref_vel = 0;
        else if (ref_vel < -mission_vel_ref)
          ref_vel = -mission_vel_ref;
      }
      else if (mission_pos_use)
      { // position regulator provides velocity reference
        bool limiter_active = rateLimitUsed or 
                              ctrlCurLeft->outLimitUsed or 
                              ctrlCurRight->outLimitUsed;
        if (balance_active)
        { // use pos in balance controller
          ctrlBalPos->outLimitUsed = limiter_active or
                                     ctrlBalVel->outLimitUsed or
                                     ctrlBal->outLimitUsed;
          ctrlBalPos->controlTick();
        }
        else
        { // not in balance, use normal position ctrl
          ctrlPos->outLimitUsed = limiter_active or
                                  ctrlVel->outLimitUsed;
          ctrlPos->controlTick();
        }
      }
    }
//     snprintf(s, MSL, "# - 3 turn do=%d, regul_line_use=%d\n", mission_turn_do, regul_line_use);
//     usb_send_str(s);
    
    if (not remoteControl)
    { // one of 4 turn possibilities
      // should PI control stop integrating (based on last tick)
      // for any dependent controllers
      bool limiter_active = ctrlTurnrate->outLimitUsed or
                            ctrlCurLeft->outLimitUsed or 
                            ctrlCurRight->outLimitUsed;
      if (mission_turn_do)
      { // fixed radius turn
        if (once)
        { // debug, running once only
          const int MSL = 100;
          char s[MSL];
          snprintf(s, MSL, "# turn control: velref=%g, angle=%g, turn_radius=%g\n", mission_vel_ref, mission_turn_angle, mission_turn_radius);
          usb_send_str(s);
          once = false;
        }
        float b_half = odoWheelBase / 2.0;
        // turn radius is always positive
        // turn angle decide which way
        if (mission_turn_angle > 0)
          ref_turnrate = ref_vel / (mission_turn_radius + b_half);
        else
          ref_turnrate = -ref_vel / (mission_turn_radius + b_half);
        // reduce velocity if turn radius is small.
        ref_vel *= 0.5 + 0.5*(mission_turn_radius - b_half)/(mission_turn_radius - b_half);
      }
      else if (regul_line_use)
      {  // turn using line edge controller
        ctrlEdge->outLimitUsed = limiter_active;
        ctrlEdge->controlTick();
      }
      else if (mission_irSensor_use > 0 and mission_wall_turn)
      {  // turn using wall follow controller
        ctrlWallTurn->outLimitUsed = limiter_active;
        ctrlWallTurn->controlTick();
      } 
      else
      { // try to keep current reference heading
        ctrlTurn->outLimitUsed = limiter_active;
        ctrlTurn->controlTick();
      }
    }
    // rate limit velocity
    if (rateLimitUse and not remoteControl)
    { // input should be rate limited - 
      // acceleration limit for a velocity controller
      float accSampleLimit = rateLimit * SAMPLETIME;
      float d0 = ref_vel - ref_vel_after_acc_limit;
        rateLimitUsed = true;
        if (d0 > accSampleLimit)
          ref_vel_after_acc_limit += accSampleLimit;
        else if (d0 < -accSampleLimit)
          ref_vel_after_acc_limit -= accSampleLimit;
        else
        { // within limit
          ref_vel_after_acc_limit = ref_vel;
          rateLimitUsed = false;
        }
    }
    else
    { // no limit - use as is
      rateLimitUsed = false;
      ref_vel_after_acc_limit = ref_vel;
    }
    // balance and velocity control
    // should balance be active
    if (balance_active)
    { // do balance control 
      // disable integrators, if either balance velocity or balance output is saturated
      ctrlBalVel->outLimitUsed = ctrlBal->outLimitUsed or 
                                 ctrlCurLeft->outLimitUsed or 
                                 ctrlCurRight->outLimitUsed;
      // velocity in balance
      ctrlBalVel->controlTick();
      // balance
      ctrlBal->controlTick();
    }
    else
    { // normal velocity control
      ctrlVel->outLimitUsed = ctrlCurLeft->outLimitUsed or 
                              ctrlCurRight->outLimitUsed;
      // normal velocity
      ctrlVel->controlTick();
    }
    // turnrate control
    ctrlTurnrate->controlTick();
    /// mix velocity and turn items
    /// left
    if (balance_active)
    {
      ref_cur[0] = u_balance - u_turnrate;
      ref_cur[1] = u_balance + u_turnrate;
    }
    else
    {
      ref_cur[0] = u_vel - u_turnrate;
      ref_cur[1] = u_vel + u_turnrate;
    }
    ctrlCurLeft->controlTick();
    ctrlCurRight->controlTick();
    // 
    // overload protect (works, if limit is set in current control)
    if (ctrlCurLeft->outLimitUsed or 
        ctrlCurRight->outLimitUsed )
      overloadCount++;
    else
      overloadCount = 0;
  }
  // 
  // is mission just finished, if so reset everything
  if ((missionState > 0 and missionStop) or (remoteCtrlEnded and missionState == 0))
  { // here once, when mission (or GUI) sets stop-flag
    motorAnkerVoltage[0] = 0;
    motorAnkerVoltage[1] = 0;
    mission_turn_do = false;
    mission_vel_ref = 0;
    mission_wall_vel_ref = 0;
    mission_wall_ref = 0;
    mission_line_ref = 0;
    mission_pos_ref = 0;
    mission_pos_use = false;
    mission_irSensor_use = false;
    controlActive = false;
    motorSetEnable(false, false);
    stopLogging();
    //misLine = NULL;
    missionLineNum = 0;
    missionStart = false;
    missionStop = false;
    usb_send_str("#** Mission stopped\r\n");
    missionState = 0;
    missionButtonCount = 0;
    remoteCtrlEnded = false;
    // enable idle collection of current data 
    // to calibrate zero current (at next mission start)
    motorPreEnabled = true;
    servo.releaseServos();
    setIRpower(false);
    lineSensorOn = false;
    if (backToNormal)
    {
      eeConfig.eePromLoadStatus(false);
      backToNormal = false;
    }
    chirpRun = false;
  }
}

void UControl::chirpReset(int logInterval)
{
  chirpAngle = 0.0;
  chirpLogInterval = logInterval;
  chirpLogQuadrant = 0;
  chirpRun = false;
  chirpMinimumFrequency  = M_PI; // rad/s
  tick = 0;
}

void UControl::chirpStart(uint8_t chirp, bool useHeading)
{
  chirpReset(logInterval);
  chirpAmplitude = chirp/100.0;
  chirpAngle = M_PI/2.0; // start at value 0 (cos(90 deg))
  chirpRun = chirp > 0;
//   chirpGyroTurnrateMax = 0;
  chirpMotorVoltMax = 0;
//   chirpGyroTurnrateMax2 = 0;
//   chirpVelMax2 = 0;
  if (chirpRun)
  { // start with 250 Hz - if log-interval=4 - in radians/sec
    chirpFrq = 2.0 * M_PI * 1000.0 / chirpLogInterval;
    // logRowCount is number of measurements in log
    // logInterval is log sample time in ms
    // make sure to reach 
    const double endFrequency = 0.5 * 2 * M_PI; // (0.5Hz)
    // calculate factor to use at every sample time (each ms)
    // see Maple calculation in regbot/matlab/chirp/chirp_calculation.ws
//     chirpFrqRate = exp(-chirpLogInterval*(chirpFrq - endFrequency)/(1000.0 * logRowsCntMax * 2 * M_PI));
//     float endTime = -log(endFrequency/chirpFrq) * logRowsCntMax * 2 * M_PI / (chirpLogInterval * (chirpFrq - endFrequency));
    // changed to update frequency at log event
    // see Maple calculation in regbot/matlab/chirp/chirp_calculation2.ws
    chirpFrqRate = exp(log(endFrequency / chirpFrq)/logRowsCntMax);
    //
    if (useHeading)
      chirpDestination = 1; // heading
    else
      chirpDestination = 0; // velocity
    // debug
    const int MSL = 150;
    char s[MSL];
    snprintf(s, MSL, "#chirp start f=%g, rate=%g, dest=%d, rows=%d, interval=%d, endTime=%g\n", 
             chirpFrq, chirpFrqRate, chirpDestination, logRowsCntMax, chirpLogInterval, endTime);
    usb_send_str(s);
    // debug end
  }
  else
    usb_send_str("#chirp off\n");
}
