 /***************************************************************************
 * definition file for the regbot.
 * The main function is the controlTick, that is called 
 * at a constant time interval (of probably 1.25 ms, see the intervalT variable)
 * 
 * The main control functions are in control.cpp file
 * 
 *   Copyright (C) 2014 by DTU                             *
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

// mission state 0 is waiting for start button
// mission state 999 is end - finished
int16_t missionState;
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
bool mission_wall_use = false;
bool mission_turn_do = false;    /// is in a turn - specified by turn radius
float mission_tr_turn = 0;       /// this is the turn angle during a turn-radius turn
float mission_turn_radius = 0.0; /// turn radius when doing a turn
float regul_bal_uvel = 0.0; /// output of balance regulator
bool backToNormal = false; // revert to flash default after hard mission
/**
  * Parameters for balance control */
bool balance_active = 0;    /// Try to use balance control
int regul_bal_use = 0;      /// use balance angle control
bool regul_balvel_use = 1;
float regul_bal_kp = 5.0;
float regul_bal_ti = 0.0;
float regul_bal_td = 0.0;
float regul_bal_alpha = 0.0;
float regul_bal_i_limit = 0.0; // integrator limit, 0=no limit
float regul_bal_u_limit = 0.0; // output limit, 0 = no limit
bool regul_bal_Lead_gyro = 1;
bool  regul_bal_LeadFwd = false;
/* Parameters for tilt velocity control */
bool regul_ss_use = 0;      /// use tilt velocity control
float regul_ss_k_tilt = 0.0;
float regul_ss_k_gyro = 0.0;
float regul_ss_k_pos = 0.0;
float regul_ss_k_vel = 0.0;
float regul_ss_k_motor = 0.0;
float regul_ss_u_limit = 0.0; // output limit, 0 = no limit
int regul_ss_step_pos = 0;
float regul_ss_step_from = 0.0;
float regul_ss_step_to = 0.0;
float regul_ss_step_time = 0.0;
/**
 * Parameters for velocity control when using balance control (controls tilt reference) */
float regul_balvel_kp = 5.0;
float regul_balvel_ti = 0.0;
float regul_balvel_td = 0.0;
float regul_balvel_alpha = 0.0;
float regul_balvel_zeta = 0.0;
float regul_balvel_i_limit = 0.0; // integrator limit, 0 = no limit
float regul_balvel_u_limit = 0.0; // output limit, 0 = no lomit
bool   regul_balVel_LeadFwd = false;
/** step parameters for balance velocity */
float regul_balvel_step_time = 0.3;
float regul_balvel_step_from = 0.0;
float regul_balvel_step_to = 0.0;
/** turn regulator */
bool regul_turn_use = false;  // use heading control (when not in a turn)
float regul_turn_kp = 5.0;
float regul_turn_ti = 0.0;
float regul_turn_td = 0.0;
float regul_turn_alpha = 0.0;
float regul_turn_i_limit = 0.0; // integrator limit, 0=no limit
bool regul_turn_LeadFwd = false;
float regul_turn_u_limit = 0.0; // output limit, 0=no limit
/** test step for turn */
float regul_turn_step_time_on = 0.2;
float regul_turn_step_time_off = 0.4; 
float regul_turn_step_val = 0.5;
float regul_turn_step_vel = 0.5;
/** line follow regulator */
bool regul_line_use = false;  // use heading control from line sensor
bool regul_line_followLeft = true;
float regul_line_kp = 5.0;
float regul_line_ti = 0.0;
float regul_line_td = 0.0;
float regul_line_alpha = 0.0;
float regul_line_i_limit = 0.0; // integrator limit, 0=no limit
bool regul_line_LeadFwd = false;
float regul_line_u_limit = 0.0; // output limit, 0=no limit
/** test step for turn */
float regul_line_step_time_on = 0.2;
float regul_line_step_from = 0.4; 
float regul_line_step_to = 0.5;
float regul_line_step_vel = 0.5;

/** wall follow regulator */
bool regul_wall_use = 0;  // use heading control from line sensor
bool regul_wall_sensor1 = true;
float regul_wall_kp = 5.0;
float regul_wall_ti = 0.0;
float regul_wall_td = 0.0;
float regul_wall_alpha = 0.0;
float regul_wall_i_limit = 0.0; // integrator limit, 0=no limit
bool regul_wall_LeadFwd = false;
float regul_wall_u_limit = 0.0; // output limit, 0=no limit
/** test step for turn */
float regul_wall_step_time = 0.2;
float regul_wall_step_from = 0.4; 
float regul_wall_step_to = 0.5;
float regul_wall_step_vel = 0.5;
bool regul_wall_do_turn = false;
bool regul_wall_sign = false;

/** result of regulator turn controll and acceleration limit
 *  used by the velocity controller */
float regul_turn_vel_reduc[2] = {0.0, 0.0};
/** acceleration limit on mission velocity, when 
 * useing balance controller */
float regul_balvel_reduc = 0.0;
/**
 * velocity PID regulator parameters */
int regul_vel_use = 0;          // use velocity control
float regul_vel_kp = 5.0;
float regul_vel_ti = 0.0;
float regul_vel_td = 0.0;
float regul_vel_alpha = 0.0;
float regul_vel_i_limit = 0.0; // integrator limit, 0=no limit
float regul_acc_limit = 100.0; // no acc limit (m/s²)
bool regul_vel_LeadFwd = false;
float regul_vel_u_limit = 0.0; // output limit, 0=no limit
/** step parameters */
float regul_vel_step_time = 0.3;
float regul_vel_step_from = 1.5;
float regul_vel_step_to = 2.5;
/** actual velocity control reference - acceleration limited */
float regul_vel_ref[2] = {0.0, 0.0}; // reference for velocity controller
float regul_vel_tot_ref[2] = {0, 0}; // added also balance without acceleration limit
/**
 * position PID regulator parameters */
int regul_pos_use = 0;          // use velocity control
float regul_pos_kp = 5.0;
float regul_pos_ti = 0.0;
float regul_pos_td = 0.0;
float regul_pos_alpha = 0.0;
float regul_pos_i_limit = 0.0; // integrator limit, 0=no limit
bool regul_pos_LeadFwd = false;
float regul_pos_u_limit = 0.0; // output limit, 0=no limit
/** step parameters */
float regul_pos_step_time = 0.3;
float regul_pos_step_from = 1.5;
float regul_pos_step_to = 2.5;
//
/**
 * velocity controller P-Lead */
float regVelELeft[2];  // input lead forward
float regVelULeft[1];  // output value of controller
float regVelMLeft[2];  // measurement input
float regVelUDLeft[2]; // output Lead
//
float regVelERight[2];  // input lead forward
float regVelURight[1];  // output value of controller
float regVelMRight[2];  // measurement input
float regVelUDRight[2]; // output Lead
// parametrs (common) for wheel velocity
float regVelParU[2];    // lead params output side
float regVelParE[2];    // lead param input side
// velocity integral part uses regVelE or regVelUD as input
float regVelUILeft[2];  // output of I-term
float regVelUIRight[2]; // output of I-term
float regVelParUI[2];
float regVelParEI[2];
/**
 * position controller P-Lead */
float regPosE[2];  // input lead forward
float regPosU[1];  // output value of controller
float regPosM[2];  // measurement input
float regPosUD[2]; // output Lead
// parametrs
float regPosParU[2];    // lead params output side
float regPosParE[2];    // lead param input side
// integral part uses regPosE or regPosUD as input
float regPosUI[2];  // output of I-term
// parameters
float regPosParUI[2];
float regPosParEI[2];

// regulator turn - P-Lead
float regTurnE[3];  // error input - added 1 for debug
float regTurnU[1];  // output value of controller
float regTurnM[3];  // measurement input - for 3 samples - added 1 for debug
float regTurnUD[2]; // lead output
float regTurnParU[2]; // param for lead (y)
float regTurnParE[2]; // param for lead (x)
// turn integral part
float regTurnUI[2]; // output of integral part
float regTurnParUI[2]; // param for I-term (y)
float regTurnParEI[2]; // param for I-term (X)
// regulator line follow - P-Lead
float regLineE[2];  // error input
float regLineU[1];  // output value of controller
float regLineM[2];  // measurement input
float regLineUD[2]; // lead output
float regLineParU[2]; // param for lead (y)
float regLineParE[2]; // param for lead (x)
// line follow integral part
float regLineUI[2]; // output of integral part
float regLineParUI[2]; // param for I-term (y)
float regLineParEI[2]; // param for I-term (X)
// regulator wall follow - P-Lead
float regWallE[2];  // error input
float regWallU[1];  // output value of controller
float regWallM[2];  // measurement input
float regWallUD[2]; // lead output
float regWallParU[2] = {0,0}; // param for lead (y)
float regWallParE[2] = {0,0}; // param for lead (x)
// wall follow integral part
float regWallUI[2]; // output of integral part
float regWallParUI[2] = {0,0}; // param for I-term (y)
float regWallParEI[2] = {0,0}; // param for I-term (X)

/// balance - Z implementation and last values
float regBalE[2];  // error input
float regBalU[1];  // output value of controller
float regBalM[2];  // measurement input
float regBalUD[2]; // output of lead
float regBalParU[2]; // param for Lead (y)
float regBalParE[2]; // param for Lead (x)
// integral part
float regBalUI[2];   // output of I-term
float regBalParUI[2];// param for I-term (y)
float regBalParEI[2];// param for I-term (x)
// NAA state controller
float posoff; // position offset at start of control
float tilt0;  // tilt value at start of control
float intgbal; // integrator value
//
/// balance velocity - Z implementation and last values
/// Lead may have complex poles
float regBalVelE[3];  // error input 
float regBalVelU[1];  // output value of controller
float regBalVelM[3];  // measurement input
float regBalVelUD[3]; // output of Lead
float regBalVelParU[3]; // param for Lead (y)
float regBalVelParE[3]; // param for Lead (x)
// integral part
float regBalVelUI[2]; // output of I-term
float regBalVelParUI[2]; // param for I-term (y)
float regBalVelParEI[2]; // param for I-term (x)
/// balance tilt velocity - Z implementation and last values
/// Lead may have complex poles
float regTVE[2];  // error input 
float regTVU[1];  // output value of controller
float regTVM[2];  // measurement input
float regTVUD[2]; // output of Lead
float regTVParU[2]; // param for Lead (y)
float regTVParE[2]; // param for Lead (x)
// integral part
float regTVUI[2]; // output of I-term
float regTVParUI[2]; // param for I-term (y)
float regTVParEI[2]; // param for I-term (x)
//
//float balTVRef;   // reference value for tilt velocity controller (for data logger)
float balTiltRef; // reference value for tilt controller (for data logger)

int mission = 0;
const int missionMax =16;
const char * missionName[missionMax] = // NB! NO spaces in name"
                            {"Velocity_step",
                              "Turn_step",
                              "Balance_test",
                              "User_mission",
                              "State_Space",
                              "edge_follow",
                              "follow_wall",
                              "Position_step",
                              "raspberry_ctrl",
                              "mission_9",
                              "hard_default",
                              "hard_follow_wall",
                              "hard_balance_step",
                              "hard_WalWal",
                              "hard_square_x4",
                              "nothing_yet"
                            };
UMissionLine * misLine = NULL;  // current mission line    "User_mission"};
UMissionLine * misLineLast = NULL;  // current mission line    "User_mission"};
uint8_t misThread = 0;  // current thread number (setwhen mis-line is set)
uint8_t misThreadLast = 0;  // last thread number (setwhen mis-line is set)

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

void sendMissionStatusChanged(bool forced)
{
  const int MSL = 120;
  char s[MSL];
  if (forced or mission != misLast or missionState != misStateLast or 
     (misLine != NULL and (misLine != misLineLast or misThread != misThreadLast)))
  {
    snprintf(s, MSL, "mis %d %d %d '%s' %d %d\r\n",
            mission, missionState, missionLineNum, 
            missionName[mission],
            sendStatusWhileRunning,
             misThread
          );
    usb_send_str(s);
    snprintf(s, MSL, "# mis %d %d  %d %d  %d %d %d\r\n", forced, mission, 
             missionState, missionStateLast, 
             misLine != NULL, misLine != misLineLast, 
             misThread != misThreadLast);
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
  // debug
//   if (missionState > 0) 
//   {
//     snprintf(s, MSL, "#time %.3f mission %d, state %d.%d, logger line %d/%d\r\n", 
//              time, mission, missionState, misLineNum, logRowCnt, logRowsCntMax);
//     usb_send_str(s);
//   }
  // debug end
}

int sendStatusControl(int8_t idx)
{
  const int MSL = 250;
  char s[MSL];
  if (idx == -1)
    idx = ++sendStatusState;
  switch (idx)
  {
    case 1:
      snprintf(s, MSL, "rgt %d %g %g %g %g %g %g %g %g %g %d %g\r\n", // use kp ti td al limit step_on step_off step_val step_vel
              regul_turn_use, regul_turn_kp, regul_turn_ti, regul_turn_td, regul_turn_alpha, regul_turn_i_limit,
              regul_turn_step_time_on, regul_turn_step_time_off, regul_turn_step_val, regul_turn_step_vel, 
              regul_turn_LeadFwd, regul_turn_u_limit
      );
      usb_send_str(s);
      break;
    case 2:
      // line follow
      snprintf(s, MSL, "rgl %d %g %g %g %g %g %g %g %g %g %d %g\r\n", // use kp ti td al limit step_on step_off step_val step_vel
              regul_line_use, regul_line_kp, regul_line_ti, regul_line_td, regul_line_alpha, regul_line_i_limit,
              regul_line_step_time_on, regul_line_step_from, regul_line_step_to, regul_line_step_vel, 
              regul_line_LeadFwd, regul_line_u_limit
      );
      usb_send_str(s);
      break;
    case 3:
      // wall follow
      snprintf(s, MSL, "rgd 1 %g %g %g %g %g %g %g %g %g %d %g %d %d\r\n", // use kp ti td al limit step_on step_off step_val step_vel
              regul_wall_kp, regul_wall_ti, regul_wall_td, regul_wall_alpha, regul_wall_i_limit,
              regul_wall_step_time, regul_wall_step_from, regul_wall_step_to, regul_wall_step_vel, 
              regul_wall_LeadFwd, regul_wall_u_limit, regul_wall_use, regul_wall_sensor1
      );
      usb_send_str(s);
      break;
    case 4:
      // velocity
      snprintf(s, MSL, "rgv %d %g %g %g %g %g %g %g %g %g %d %d %g\r\n",
              regul_vel_use, regul_vel_kp, regul_vel_ti, regul_vel_td, regul_vel_alpha, regul_vel_i_limit,
              regul_vel_step_time, regul_vel_step_from, regul_vel_step_to, 
              regul_acc_limit, 0, regul_vel_LeadFwd, max_motor_voltage
      );
      usb_send_str(s);
      break;
    case 5:
      // position
      snprintf(s, MSL, "rgp %d %g %g %g %g %g %g %g %g %d %g\r\n",
              regul_pos_use, regul_pos_kp, regul_pos_ti, regul_pos_td, regul_pos_alpha, regul_pos_i_limit,
              regul_pos_step_time, regul_pos_step_from, regul_pos_step_to, 
              regul_pos_LeadFwd, regul_pos_u_limit
      );
      usb_send_str(s);
      break;
    case 6:
      // speed and turn Z-expression
      snprintf(s, MSL,  "rgvz %g %g %g\r\n"
                        "rgvzi %g %g %g\r\n" // denom[1], numer[0] numer[1]
                        "rgtz %g %g %g\r\n"
                        "rgtzi %g %g %g\r\n", // denom[1], numer[0] numer[1]
                        regVelParU[1], regVelParE[0], regVelParE[1],
                        regVelParUI[1], regVelParEI[0], regVelParEI[1],
                        regTurnParU[1], regTurnParE[0], regTurnParE[1],
                        regTurnParUI[1], regTurnParEI[0], regTurnParEI[1]
      );
      usb_send_str(s);
      break;
    case 7:
      // line follow Z-expression
      snprintf(s, MSL,  "rglz %g %g %g\r\n"
                        "rglzi %g %g %g\r\n",
                        regLineParU[1], regLineParE[0], regLineParE[1],
                        regLineParUI[1], regLineParEI[0], regLineParEI[1]
      );
      usb_send_str(s);
      break;
    case 8:
      // position control Z-expression
      snprintf(s, MSL,  "rgpz %g %g %g\r\n"
              "rgpzi %g %g %g\r\n",
              regPosParU[1], regPosParE[0], regPosParE[1],
              regPosParUI[1], regPosParEI[0], regPosParEI[1]
      );
      usb_send_str(s);
      break;
    case 9:
      // wall follow Z-expression
      snprintf(s, MSL,  "rgdz %g %g %g\r\n"
              "rgdzi %g %g %g\r\n",
              regWallParU[1], regWallParE[0], regWallParE[1],
              regWallParUI[1], regWallParEI[0], regWallParEI[1]
      );
      usb_send_str(s);
      break;
    case 10:
      // balance control - and balance velocity step
      snprintf(s, MSL, "rgb %d %g %g %g %g %g %g %g %g %d %d %g %d\r\n",
              regul_bal_use, regul_bal_kp, regul_bal_ti, regul_bal_td, regul_bal_alpha, regul_bal_i_limit,
              regul_balvel_step_time, regul_balvel_step_from, regul_balvel_step_to, 
              regul_balvel_use, regul_bal_LeadFwd, regul_bal_u_limit, regul_bal_Lead_gyro
      );
      usb_send_str(s);
      break;
    case 11:
      // balance state space model
      snprintf(s, MSL, "rgs %d %g %g %g %g %g %g %g %g %g %d\r\n",
              regul_ss_use, regul_ss_k_tilt, regul_ss_k_gyro, regul_ss_k_pos, regul_ss_k_motor, 
              regul_ss_k_vel, regul_ss_u_limit, regul_ss_step_time, regul_ss_step_from, regul_ss_step_to, regul_ss_step_pos
      );
      usb_send_str(s);
      break;
    case 12:
      // balance mission velocity
      snprintf(s, MSL, "rgmv %g %g %g %g %g %g %d %g\r\n",
              regul_balvel_kp, regul_balvel_ti, regul_balvel_td, regul_balvel_alpha, regul_balvel_zeta, 
              regul_balvel_i_limit, regul_balVel_LeadFwd, regul_balvel_u_limit
      );
      usb_send_str(s);
      break;
    case 13:
      // balance and speed Z-expression
      snprintf(s, MSL,  "rgbz %g %g %g\r\n"
              "rgbzi %g %g %g\r\n" // denom[1], numer[0] numer[1]
              "rgmvz %g %g %g %g %g\r\n"
              "rgmvzi %g %g %g\r\n", // denom[1], numer[0] numer[1]
              regBalParU[1], regBalParE[0], regBalParE[1],
              regBalParUI[1], regBalParEI[0], regBalParEI[1],
              regBalVelParU[1], regBalVelParU[2], regBalVelParE[0], regBalVelParE[1], regBalVelParE[2],
              regBalVelParUI[1], regBalVelParEI[0], regBalVelParEI[1]
      );
      usb_send_str(s);
      break;
    default:
      sendStatusState = 0;
  }
  return sendStatusState;
}

/**
 * Step on velocity, as specified by GUI
 * parameter estimate mission (should use fast logging) */
void mission_gui_vel_step()
{
  const int MSL = 60;
  char s[MSL];
  switch (missionState)
  {
    case 0: // wait for start button
      if (button or missionStart)
      { // goto next state
        time = 0.0;
        // ensure main log options are on
//         logRowFlags[LOG_MOTV] = true;    // orderd anchor voltage before PWM
//         logRowFlags[LOG_WHEELVEL] = true;  // wheel velocity in rad/s
        mission_vel_ref = 0.0;
        mission_turn_do = false;
        missionState = 1;
      }
      break;
    case 1:
      if (time > 1.0) // wait for finger away from button
      { // go to next state
        time = 0.0;
        clearPose();
        startLogging(logInterval, true); 
        mission_vel_ref = regul_vel_step_from;
        // debug
        snprintf(s, MSL, "# mission vel ref from %.2f rad/s\r\n", mission_vel_ref);
        usb_send_str(s);
        // debug end
        motorSetEnable(true, true);
        missionState = 2;
      }
      break;
    case 2:
      if (time > regul_vel_step_time or not loggerLogging())
      { // go to next state
        missionState = 4;
        mission_vel_ref = regul_vel_step_to;
        // debug
        snprintf(s, MSL, "# mission vel ref to   %.2f rad/s at %.3f\r\n", mission_vel_ref, time);
        usb_send_str(s);
        // debug end
      }
      break;
    case 4:
      // continue until log buffer is full
      if (not loggerLogging())
      { // finished logging
        // finish
        mission_vel_ref = 0.0;
        missionState = 999;
        // debug
        snprintf(s, MSL, "# mission vel ref to   %.2f rad/s at %.3f\r\n", mission_vel_ref, time);
        usb_send_str(s);
        // debug end
      }
      break;
    default:
      // go back to - try again.
      missionStop = true;
      break;
  }
}

/**
 * Step on position, as specified by GUI
 * parameter estimate mission (should use fast logging) */
void mission_gui_pos_step()
{
  const int MSL = 60;
  char s[MSL];
  switch (missionState)
  {
    case 0: // wait for start button
      if (button or missionStart)
      { // goto next state
        time = 0.0;
        // ensure main log options are on
        //         logRowFlags[LOG_MOTV] = true;    // orderd anchor voltage before PWM
        //         logRowFlags[LOG_WHEELVEL] = true;  // wheel velocity in rad/s
        mission_pos_ref = 0.0;
        mission_turn_do = false;
        regul_vel_ref[0] = 0.0;
        regul_vel_ref[1] = 0.0;
        missionState = 1;
        misStartDist = distance;
      }
      break;
    case 1:
      if (time > 1.0) // wait for finger away from button
      { // go to next state
        time = 0.0;
        clearPose();
        misStartDist = distance;
        startLogging(logInterval, true); 
        mission_pos_ref = regul_pos_step_from;
        // debug
        snprintf(s, MSL, "# mission pos ref from %.2f rad/s\r\n", mission_pos_ref);
        usb_send_str(s);
        // debug end
        motorSetEnable(true, true);
        missionState = 2;
      }
      break;
    case 2:
      if (time > regul_pos_step_time or not loggerLogging())
      { // go to next state
        missionState = 4;
        mission_pos_ref = regul_pos_step_to;
        // debug
        snprintf(s, MSL, "# mission pos ref to   %.2f rad/s at %.3f\r\n", mission_pos_ref, time);
        usb_send_str(s);
        // debug end
      }
      break;
    case 4:
      // continue until log buffer is full
      if (not loggerLogging())
      { // finished logging
        // finish
        mission_pos_ref = 0.0;
        missionState = 999;
        // debug
        snprintf(s, MSL, "# mission pos ref to   %.2f rad/s at %.3f\r\n", mission_pos_ref, time);
        usb_send_str(s);
        // debug end
      }
      break;
    default:
      // go back to - try again.
      missionStop = true;
      break;
  }
}
/**
 * Step on position, as specified by GUI
 * parameter estimate mission (should use fast logging) */
void mission_hard_coded()
{
  const int MSL = 60;
  char s[MSL];
  switch (missionState)
  {
    case 0: // wait for start button
      if (button or missionStart)
      { // goto next state
        time = 0.0;
        missionState = 1;
        missionStart = false;
        snprintf(s, MSL, "# loading hard coded mission %d (idx=%d)\r\n", mission, mission - 10);
        usb_send_str(s);
      }
      break;
    case 1:
      if (time > 0.5) // wait for finger away from button
      { // go to next state
        bool isOK;
        bool back;
        time = 0.0;
        missionState = 0;
        back = mission > 10;
        isOK = eeConfig.hardConfigLoad(mission - 10, false);
        snprintf(s, MSL, "# implement as mission %d\r\n", mission);
        usb_send_str(s);
        // start the newly loaded mission
        if (isOK)
        {
          missionStart = true;
          backToNormal = back;
        }
      }
      break;
    default:
      break;
  }
}
/**
 * Step on turn, as specified by GUI
 * parameter estimate mission (should use fast logging) */
void mission_gui_turn_step()
{
  switch (missionState)
  {
    case 0: // wait for start button
      if (button or missionStart)
      { // goto next state
        time = 0.0;
        clearPose(); 
        // ensure main log options are on
//         logRowFlags[LOG_TURNRATE] = true;  // robot turnrate
//         logRowFlags[LOG_MOTV_REF] = true;  // u(t) - ref for motor
//         logRowFlags[LOG_POSE] = true;      // robot pose (heading)
        missionState = 1; 
        mission_turn_ref = 0.0;
        mission_turn_do = false;
        mission_vel_ref = 0.0;
      }
      break;
    case 1:
      if (time > 1.0) // wait for finger away from button
      { // go to next state
        time = 0.0;
        clearPose();
        startLogging(logInterval, true); 
        mission_vel_ref = regul_turn_step_vel;
        mission_turn_ref = 0.0;
        motorSetEnable(true, true);
        missionState = 2;
      }
      break;
    case 2: // straight
      if (time > regul_turn_step_time_on  or not loggerLogging())
      { // go to turn
        missionState = 4;
        mission_turn_ref = regul_turn_step_val;
      }
      break;
    case 4: // turn
      if (time > regul_turn_step_time_off  or not loggerLogging())
      { // go to next state - no more turn
        missionState = 6;
        mission_turn_ref = 0.0;
      }
      break;
    case 6: // straight
      // continue until log buffer is full
      if (not loggerLogging())
      { // finished logging so stop
        mission_turn_ref = 0.0;
        mission_vel_ref = 0.0;
        missionState = 998;
      }
      break;
    default: // stop
      missionStop = true;
      break;
  }
}

/**
 * Step on edge follow, as specified by GUI
 *   */
void mission_gui_line_step()
{
  switch (missionState)
  {
    case 0: // wait for start button
      if (button or missionStart)
      { // goto next state
        time = 0.0;
        clearPose(); 
        missionState = 1; 
        // set start reference values
        mission_line_ref = regul_line_step_from;
        mission_line_LeftEdge = regul_line_followLeft;
        mission_vel_ref = 0.0;
      }
      break;
    case 1:
      if (time > 1.0) // wait 1 second for finger away from button
      { // go to next state
        time = 0.0;
        clearPose();
        startLogging(logInterval, true); 
        mission_vel_ref = regul_line_step_vel;
        motorSetEnable(true, true);
        missionState = 2;
      }
      break;
    case 2: // follow line
      if (time > regul_line_step_time_on or not loggerLogging())
      { // go to next reference position
        missionState = 6;
        mission_line_ref = regul_line_step_to;
      }
      break;
    case 6: // straight
      // continue until log buffer is full
      if (not loggerLogging())
      { // finished logging so stop
        mission_line_ref = 0.0;
        mission_line_LeftEdge = true;
        missionState = 998;
      }
      break;
    default: // stop
      missionStop = true;
      break;
  }
}

//////////////////////////////////////////////////////////////////////////////

/**
 * Step on wall follow, as specified by GUI
 *   */
void mission_gui_wall_step()
{
  switch (missionState)
  {
    case 0: // wait for start button
      if (button or missionStart)
      { // goto next state
        time = 0.0;
        clearPose(); 
        missionState = 1; 
        // set start reference values
        mission_wall_ref = regul_wall_step_from;
        mission_wall_turn = regul_wall_sensor1;
        mission_vel_ref = 0.0;
        mission_wall_use = true;
        //mission_wall_sign = regul_wall_sign;
      }
      break;
    case 1:
      if (time > 1.0) // wait 1 second for finger away from button
      { // go to next state
        time = 0.0;
        clearPose();
        startLogging(logInterval, true); 
        mission_wall_ref = regul_wall_step_from;
        mission_vel_ref = regul_wall_step_vel;
        motorSetEnable(true, true);
        missionState = 2;
      }
      break;
    case 2: // follow line
      if (time > regul_wall_step_time  or not loggerLogging())
      { // go to next reference position
        missionState = 6;
        mission_wall_ref = regul_wall_step_to;
      }
      break;
    case 6: // straight
      // continue until log buffer is full
      if (not loggerLogging())
      { // finished logging so stop
        mission_wall_ref = 0.0;
        mission_vel_ref = 0.0;
        //mission_wall_sensor1 = true;
        missionState = 998;
      }
      break;
    default: // stop
      missionStop = true;
      break;
  }
}

///////////////////////////////////////////////////////////////////////

/**
 * run mission from user lines */
void user_mission()
{
  float endTime;
  switch (missionState)
  {
    case 0: // wait for start button
      if (button or missionStart)
      { // goto next state
        time = 0.0;
        missionState = 1;
      }
      break;
    case 1: // wait to allow release of button
      if (time > 1.0)
      { // init next state
        if (userMission.startMission())
        {
          missionState = 2;
          //usb_send_str("# user_mission -> state 2\n");
        }
        else
        {  // no user defined lines
          missionState = 999;
          break;
        }
        motorSetEnable(true, true);
      }
      break;
      case 2:
        // run user mission lines
        if (button)
          missionState = 998;
        else
        {
          bool theEnd = userMission.testFinished();
          if (theEnd or missionStop)
          {
            missionState=998;
            //usb_send_str("# user_mission -> the end!\n");
          }
        }
        break;
      case 998:
        // stop (start) button pressed
        // wait for it to be released
        mission_vel_ref = 0;
        endTime = time + 0.5;
        if (not button and time > endTime)
          missionState = 999;
      default:
        missionStop = true;
        break;
  }
}


/**
 * parameter estimate mission with slow logging */
void balance_test()
{
  switch (missionState)
  {
    case 0: // wait for start button
      if (button or missionStart)
      { // goto next state
        time = 0.0;
        missionState = 1; 
        balance_active = 1;
      }
      break;
    case 1: // wait to allow release of button
      if (time > 1.0)
      { // goto next state
        time = 0.0;
        clearPose();
        mission_vel_ref = regul_balvel_step_from;
        startLogging(logInterval, true); 
        missionState = 2; 
        // and start the engines
        motorSetEnable(true, true);
      }
      break;
    case 2:
      // continue until log buffer is full
      if (not loggerLogging() or time > regul_balvel_step_time)
      { // finished logging
        // finish
        mission_vel_ref = regul_balvel_step_to;
        missionState = 3;
      }
      break;
    case 3:
      // continue until log buffer is full
      if (not loggerLogging())
      { // finished logging
        // finish
        missionState = 999;
      }
      break;
    default:
      missionStop = true;
      break;
  }
}

/**
 * parameter estimate mission with slow logging */
void balance_ss_test()
{
  switch (missionState)
  {
    case 0: // wait for start button
      if (button or missionStart)
      { // goto next state
        time = 0.0;
        missionState = 1; 
        balance_active = 1;
      }
      break;
    case 1: // wait to allow release of button
      if (time > 1.0)
      { // goto next state
        time = 0.0;
        usb_send_str("# State Space controller is not implemented\r\n");
        clearPose();
        switch (regul_ss_step_pos)
        {
          case 0:
            mission_vel_ref = 0.0;
            mission_pos_ref = regul_ss_step_from;
            mission_vel_ref_ref = 0.0;
            break;
          case 1:
            mission_vel_ref = regul_ss_step_from;
            mission_pos_ref = 0.0;
            mission_vel_ref_ref = 0.0;
            break;
          default:
            mission_vel_ref = 0.0;
            mission_pos_ref = 0.0;
            mission_vel_ref_ref = regul_ss_step_from;
            break;
        }
        startLogging(logInterval, true); 
        missionState = 2; 
        // and start the engines
        motorSetEnable(true, true);
      }
      break;
    case 2:
      // continue until log buffer is full
      if (not loggerLogging() or time > regul_ss_step_time)
      { // finished logging
        switch (regul_ss_step_pos)
        {
          case 0:
            mission_vel_ref = 0.0;
            mission_pos_ref = regul_ss_step_to;
            mission_vel_ref_ref = 0.0;
            break;
          case 1:
            mission_vel_ref = regul_ss_step_to;
            mission_pos_ref = 0.0;
            mission_vel_ref_ref = 0.0;
            break;
          default:
            mission_vel_ref = 0.0;
            mission_pos_ref = 0.0;
            mission_vel_ref_ref = regul_ss_step_to;
            break;
        }
        missionState = 3;
      }
      break;
    case 3:
      // continue until log buffer is full
      if (not loggerLogging())
      { // finished logging
        // finish
        missionState = 999;
      }
      break;
    default:
      missionStop = true;
      break;
  }
}


// void regulatorInit1(float kp, float ti, float td, float al, float * zu, float * ze)
// {
//   const float T = 0.001;
//   if (ti == 0.0 and td == 0.0)
//   { // P-regulator
//     zu[1] = 0.0;
//     zu[2] = 0.0;
//     ze[0] = kp;
//     ze[1] = 0.0;
//     ze[2] = 0.0;
//   }
//   else if (td == 0.0)
//   {
//     zu[0] = 2 * ti;
//     zu[1] = -1;
//     zu[2] = 0;
//     ze[0] = kp * (T + 2 * ti) / zu[0];
//     ze[1] = kp * (T - 2 * ti) / zu[0];
//     ze[2] = 0;
//   }
//   else if (ti == 0.0)
//   {
//     zu[0] = 2 * al * td + T;
//     zu[1] = T - 2 * al * td;
//     zu[2] = 0;
//     ze[0] = kp * (T + 2 * td) / zu[0];
//     ze[1] = kp * (T - 2 * td) / zu[0];
//     ze[2] = 0;
//   }
//   else
//   {
//     zu[0] =  4 * al * td * ti + 2 * T * ti;
//     zu[1] = -8 * al * td * ti / zu[0];
//     zu[2] = (4 * al * td * ti - 2 * T * ti) / zu[0];
//     ze[0] = kp * (T * (T + 2 * td + 2 * ti) + 4  * td * ti) / zu[0];
//     ze[1] = kp * (2* T * T - 8 * td * ti) / zu[0];
//     ze[2] = kp * (T * (T - 2* td - 2* ti) + 4 * td * ti) / zu[0];
//   }
//   regVelELeft[1] = 0;
//   regVelELeft[2] = 0;
//   regVelERight[1] = 0;
//   regVelERight[2] = 0;
//   regVelUDLeft[1] = 0;
//   regVelUDLeft[2] = 0;
//   regVelERight[1] = 0;
//   regVelERight[2] = 0;
// }

/**
 * Initialize discrete regulators from continious parameters
 * input:
 * \param kp is proportional gain
 * \param ti is time constant for zero in I-term
 * \param td is time constant in lead zero
 * \param al is relative position of pole in lead term
 * output:
 * \param zu is pointer to parameters for denominator in z-expression for lead term
 * \param ze is pointer to parameters for numerator for z-expression for lead term
 * \param zui is pointer to parameters for denominator in z-expression for I-term
 * \param zei is pointer to parameters for numerator for z-expression for I-term
 */
// void regulatorInit2(float kp, float ti, float td, float al, float * zu, float * ze, float * zui, float * zei)
// {
//   const float T = 0.001;
//   if (td <= 0.0 or al <= 0.0)
//   { // P-regulator
//     zu[0] = 1.0; // denominator
//     zu[1] = 0.0;
//     ze[0] = kp;  // numerator
//     ze[1] = 0.0;
//   }
//   else
//   { // include lead term
//     /*
//      * Gzd2:=expand(Gzd*z^(-1));
//      *                                 2 al td   T
//      *                   2 al td + T - ------- + -
//      *                                    z      z
//      * Gzn2:=expand(Gzn*z^(-1));
//      *                                 T   2 td
//      *                      T + 2 td + - - ----
//      *                                 z    z  
//      */
//     zu[0] = 2 * al * td + T;           // denominator (this tern is normalized away)
//     zu[1] = (T - 2 * al * td) / zu[0];
//     ze[0] = kp * (T + 2 * td) / zu[0]; // numerator
//     ze[1] = kp * (T - 2 * td) / zu[0];
//   }
//   if (ti > 0.0)
//   { // engage the integrator
//     /*
//      * Gzd2:=expand(Gzd*z^(-1));
//      *                                 2 ti
//      *                          2 ti - ----
//      *                                  z  
//      * Gzn2:=expand(Gzn*z^(-1));
//      *                                 T   2 ti
//      *                      T + 2 ti + - - ----
//      *                                 z    z  
//      */
//     zui[0] = 2 * ti; // denominator
//     zui[1] = -1;
//     zei[0] = (T + 2 * ti) / zui[0]; // numerator
//     zei[1] = (T - 2 * ti) / zui[0];
//   }
//   else
//   { // no added value from integrator
//     zui[0] = 1;
//     zui[1] = 0;
//     zei[0] = 0;
//     zei[1] = 0;
//   }
// }

/**
 * Initialize discrete regulators from continious parameters
 * input:
 * \param kp is proportional gain
 * \param ti is time constant for zero in I-term
 * \param td is time constant in lead zero
 * \param al is relative position of pole in lead term
 * \param zeta is damping of complex pole pair
 * output:
 * \param zu is pointer to parameters for denominator in z-expression for lead term
 * \param ze is pointer to parameters for numerator for z-expression for lead term
 * \param zui is pointer to parameters for denominator in z-expression for I-term
 * \param zei is pointer to parameters for numerator for z-expression for I-term
 */
void regulatorInitComplexLead(float kp, float ti, float td, float al, float zeta, float * zu, float * ze, float * zui, float * zei)
{
  const float T = 0.001;
  if (td <= 0.0 or al <= 0.0)
  { // P-regulator
    zu[0] = 1.0; // denominator
    zu[1] = 0.0;
    ze[0] = kp;  // numerator
    ze[1] = 0.0;
  }
  else if (zeta <= 0.0)
  { // include lead term
    /*
      * Gzd2:=expand(Gzd*z^(-1));
      *                                 2 al td   T
      *                   2 al td + T - ------- + -
      *                                    z      z
      * Gzn2:=expand(Gzn*z^(-1));
      *                                 T   2 td
      *                      T + 2 td + - - ----
      *                                 z    z  
      */
    zu[0] = 2 * al * td + T; // denominator 
    zu[1] = (T - 2 * al * td) / zu[0];
    ze[0] = kp * (T + 2 * td) / zu[0]; // numerator
    ze[1] = kp * (T - 2 * td) / zu[0];
  }
  else
  { // complex lead term
    /* Denominator
     * Gzd2:=expand(Gzd*z^(-2));
     *                                             2   2      2
     *                            2   2    2   8 al  td    2 T 
     *       4 al td zeta T + 4 al  td  + T  - --------- + ----
     *                                             z        z  
     * 
     *                                 2   2    2
     *            4 al td zeta T   4 al  td    T 
     *          - -------------- + --------- + --
     *                   2             2        2
     *                  z             z        z 
     * Numerator
     *    Gzn2:=expand(Gzn*z^(-2));
     *                              2    2            
     *              2            2 T    T    2 T td
     *        Kp ( T  + 2 T td + ---- + -- - ------ )
     *                            z      2      2    
     *                                  z      z     
     */
    zu[0] = 4 * al * td * zeta * T + 4 * al*al * td*td + T*T; // denominator (k)
    zu[1] = (2*T*T - 8 * al*al * td*td)/zu[0];
    zu[2] = (-4* al * td * zeta * T + 4 * al*al * td*td + T*T)/zu[0];
    ze[0] = kp * (T*T + 2 * T * td) / zu[0]; // numerator
    ze[1] = kp * (2 * T*T) / zu[0];
    ze[2] = kp * (T*T - 2 * T * td) / zu[0];
  }
  if (ti > 0.0)
  { // engage the integrator
    /*
     * GC(s) = 1/(tau_i s)
     * Gzd2:=expand(Gzd*z^(-1));
     *                                 2 ti
     *                          2 ti - ----
     *                                  z  
     * Gzn2:=expand(Gzn*z^(-1));
     *                                 T
     *                             T + -
     *                                 z
     * 
     * 
     */
    zui[0] = 2 * ti; // deniminator
    zui[1] = -1;
    zei[0] = (T) / zui[0]; // numerator
    zei[1] = (T) / zui[0];
  }
  else
  { // no added value from integrator
    zui[0] = 1;
    zui[1] = 0;
    zei[0] = 0;
    zei[1] = 0;
  }
}

/**
 * simpel filter 2 order */
void filt1order(float * x, float * y, float * parX, float * parY)
{
    y[0] = -parY[1] * y[1] + 
            parX[0] * x[0] + parX[1] * x[1];
}

/**
 * simpel filter 2 order */
void filt2order(float * x, float * y, float * parX, float * parY)
{
  y[0] = -parY[1] * y[1] - parY[2] * y[2] + 
          parX[0] * x[0] + parX[1] * x[1] + parX[2] * x[2];
}



/**
 * Do control of wheel velocity
 * \param ref is desired velocity (rad/sec)
 * \param measured is measured velocity (rad/sec)
 * \param left if trur, then use left dataset, else use right */
void regulWheelVel(float ref, float measured, bool left)
{
  //regVelELeft, regVelUDLeft, regVelUILeft,  regVelULeft
  if (left)
  { 
    if (regul_vel_LeadFwd)
    { // calculate new error
      regVelELeft[0] = (ref - measured) * regul_vel_kp;
      // lead in fwd chain
      // filt 1 order ( input array, output array, x-params (numerator), y-params(denom))
      filt1order(regVelELeft, regVelUDLeft, regVelParE, regVelParU);
      // integral part - output of lead is input to I
      filt1order(regVelUDLeft, regVelUILeft, regVelParEI, regVelParUI);      
    }
    else
    { // lead in measured path
      regVelMLeft[0] = measured;
      filt1order(regVelMLeft, regVelUDLeft, regVelParE, regVelParU);
      // calculate error
      regVelELeft[0] = (ref - regVelUDLeft[0]) * regul_vel_kp;
      // integral part - output of lead is input to I
      filt1order(regVelELeft, regVelUILeft, regVelParEI, regVelParUI);
      // make one step older
      regVelMLeft[1] = regVelMLeft[0];
    }
    // limit I
    if (regVelUILeft[0] > regul_vel_i_limit)
      regVelUILeft[0] = regul_vel_i_limit;
    else if (regVelUILeft[0] < -regul_vel_i_limit)
      regVelUILeft[0] = -regul_vel_i_limit;
    // sum P-lead and I them
    if (regul_vel_LeadFwd)
      regVelULeft[0] = regVelUDLeft[0] + regVelUILeft[0];
    else
      regVelULeft[0] = regVelELeft[0] + regVelUILeft[0];
    // make one step older
    regVelELeft[1] = regVelELeft[0];
    regVelUDLeft[1] = regVelUDLeft[0];
    regVelUILeft[1] = regVelUILeft[0];      
  }
  else
  {
    if (regul_vel_LeadFwd)
    { // calculate new error
      regVelERight[0] = (ref - measured) * regul_vel_kp;
      // lead in fwd chain
      // filt 1 order ( input array, output array, x-params (numerator), y-params(denom))
      filt1order(regVelERight, regVelUDRight, regVelParE, regVelParU);
      // integral part - output of lead is input to I
      filt1order(regVelUDRight, regVelUIRight, regVelParEI, regVelParUI);
    }
    else
    { // lead in measured path
      regVelMRight[0] = measured;
      filt1order(regVelMRight, regVelUDRight, regVelParE, regVelParU);
      // calculate error
      regVelERight[0] = (ref - regVelUDRight[0]) * regul_vel_kp;
      // integral part - output of lead is input to I
      filt1order(regVelERight, regVelUIRight, regVelParEI, regVelParUI);
      // make one step older
      regVelMRight[1] = regVelMRight[0];
    }
    // limit I
    if (regVelUIRight[0] > regul_vel_i_limit)
      regVelUIRight[0] = regul_vel_i_limit;
    else if (regVelUIRight[0] < -regul_vel_i_limit)
      regVelUIRight[0] = -regul_vel_i_limit;
    // sum P-lead and I them
    if (regul_vel_LeadFwd)
      regVelURight[0] = regVelUDRight[0] + regVelUIRight[0];
    else
      regVelURight[0] = regVelERight[0] + regVelUIRight[0];
    // make one step older
    regVelERight[1] = regVelERight[0];
    regVelUDRight[1] = regVelUDRight[0];
    regVelUIRight[1] = regVelUIRight[0];      
  }
}

/**
 * Do control of wheel velocity adjustment for turning
 * \param ref is desired heading
 * \param measured is measured heading
 *  */
void regulSteer(float ref, float measured)
{
  //regVelELeft, regVelUDLeft, regVelUILeft,  regVelULeft
  if (regul_turn_LeadFwd)
  { // calculate new error
    regTurnE[0] = ref - measured; // both angles
    if (regTurnE[0] > M_PI)
      regTurnE[0] -= 2.0 * M_PI;
    else if (regTurnE[0] < - M_PI)
      regTurnE[0] += 2.0 * M_PI;
    regTurnE[0] *= regul_turn_kp;
    // lead in fwd chain
    // filt 1 order ( input array, output array, x-params (numerator), y-params(denom))
    filt1order(regTurnE, regTurnUD, regTurnParE, regTurnParU);
    // integral part - output of lead is input to I
    filt1order(regTurnUD, regTurnUI, regTurnParEI, regTurnParUI);
  }
  else
  { // lead in measured path
    regTurnM[0] = measured;
    // test if the measured has changed rotation since the history version
    if (regTurnM[0] - regTurnM[1] > M_PI)
    { //adjust old valuse to follow measured
      regTurnM[1] += 2.0 * M_PI;
      regTurnUD[1] += 2.0 * M_PI;
    }
    else if (regTurnM[0] - regTurnM[1] < -M_PI)
    { //adjust old valuse to follow measured
      regTurnM[1] -= 2.0 * M_PI;
      regTurnUD[1] -= 2.0 * M_PI;
    }
    // debug
    regTurnM[2] = regTurnM[1];
    // debug end
    // make Lead filtering
    filt1order(regTurnM, regTurnUD, regTurnParE, regTurnParU);
    // ensure also that measured and reference are in same rotation
    regTurnE[0] = ref - measured;
    if (regTurnE[0] > M_PI)
      ref -= 2.0 * M_PI;
    else if (regTurnE[0] < - M_PI)
      ref += 2.0 * M_PI;
    // then the difference should be valid also after the lead filter
    regTurnE[2] = (ref - regTurnUD[0]);
    // debug
//     if (regTurnE[0] > M_PI or regTurnE[0] < -M_PI)
//     {
//       const int MSL = 180;
//       char s[MSL];
//       snprintf(s, MSL, "# angle wrap error: ref %f measured %f regE %f UD %f\n", ref, measured, regTurnE[0], regTurnUD[0]);
//       usb_send_str(s);
//     }
    regTurnE[0] = regTurnE[2] * regul_turn_kp;
    // integral part - output of lead is input to I
    filt1order(regTurnE, regTurnUI, regTurnParEI, regTurnParUI);
    // make one step older
    regTurnM[1] = regTurnM[0];
  }
  // limit I
  if (regul_turn_i_limit > 1e-5)
  { // integration is to be limited
    if (regTurnUI[0] > regul_turn_i_limit)
      regTurnUI[0] = regul_turn_i_limit;
    else if (regTurnUI[0] < -regul_turn_i_limit)
      regTurnUI[0] = -regul_turn_i_limit;
  }
  // sum P-lead/P and I term
  if (regul_turn_LeadFwd)
    regTurnU[0] = regTurnUD[0] + regTurnUI[0];
  else
    regTurnU[0] = regTurnE[0] + regTurnUI[0];
  //
  if (regul_turn_u_limit > 0.001)
  { // output limit enabled
    if (regTurnU[0] > regul_turn_u_limit)
      regTurnU[0] = regul_turn_u_limit;
    else if (regTurnU[0] < -regul_turn_u_limit)
      regTurnU[0] = -regul_turn_u_limit;
  }
  // make one step older
  regTurnE[1] = regTurnE[0];
  regTurnUD[1] = regTurnUD[0];
  regTurnUI[1] = regTurnUI[0];      
}

/**
 * Do line follow control
 * \param ref is desired position (cm)
 * \param measured is measured position (cm)
 *  */
void regulLine(float ref, float measured)
{
  if (regul_line_LeadFwd)
  { // calculate new error
    regLineE[0] = ref - measured; // both angles
    regLineE[0] *= regul_line_kp;
    // lead in fwd chain
    // filt 1 order ( input array, output array, x-params (numerator), y-params(denom))
    filt1order(regLineE, regLineUD, regLineParE, regLineParU);
    // integral part - output of lead is input to I
    filt1order(regLineUD, regLineUI, regLineParEI, regLineParUI);
  }
  else
  { // lead in measured path
    regLineM[0] = measured;
    // make Lead filtering
    filt1order(regLineM, regLineUD, regLineParE, regLineParU);
    // ensure also that measured and reference are in same rotation
    regLineE[0] = ref - measured;
    // then the difference should be valid also after the lead filter
    regLineE[0] = (ref - regLineUD[0]) * regul_line_kp;
    // integral part - output of lead is input to I
    filt1order(regLineE, regLineUI, regLineParEI, regLineParUI);
    // make one step older
    regLineM[1] = regLineM[0];
  }
  // limit I
  if (regul_line_i_limit > 1e-5)
  { // integration is to be limited
    if (regLineUI[0] > regul_line_i_limit)
      regLineUI[0] = regul_line_i_limit;
    else if (regLineUI[0] < -regul_line_i_limit)
      regLineUI[0] = -regul_line_i_limit;
  }
  // sum P-lead/P and I term
  if (regul_line_LeadFwd)
    regLineU[0] = regLineUD[0] + regLineUI[0];
  else
    regLineU[0] = regLineE[0] + regLineUI[0];
  //
  if (regul_line_u_limit > 0.001)
  { // output limit enabled
    if (regLineU[0] > regul_line_u_limit)
      regLineU[0] = regul_line_u_limit;
    else if (regLineU[0] < -regul_line_u_limit)
      regLineU[0] = -regul_turn_u_limit;
  }
  // make one step older
  regLineE[1] = regLineE[0];
  regLineUD[1] = regLineUD[0];
  regLineUI[1] = regLineUI[0];      
}

///////////////////////////////////////////////////////
/**
 * Do position control
 * \param ref is desired position (m) 
 * \param measured is measured position (m) as noted in pose calculation
 *  */
void regulPosition(float ref, float measured)
{
  if (regul_pos_LeadFwd)
  { // calculate new error
    regPosE[0] = ref - measured; // both angles
    regPosE[0] *= regul_pos_kp;
    // lead in fwd chain
    // filt 1 order ( input array, output array, x-params (numerator), y-params(denom))
    filt1order(regPosE, regPosUD, regPosParE, regPosParU);
    // integral part - output of lead is input to I
    filt1order(regPosUD, regPosUI, regPosParEI, regPosParUI);
  }
  else
  { // lead in measured path
    regPosM[0] = measured;
    // make Lead filtering
    filt1order(regPosM, regPosUD, regPosParE, regPosParU);
    // ensure also that measured and reference are in same rotation
    //regPosE[0] = ref - measured;
    // then the difference should be valid also after the lead filter
    regPosE[0] = (ref - regPosUD[0]) * regul_pos_kp;
    // integral part - output of lead is input to I
    filt1order(regPosE, regPosUI, regPosParEI, regPosParUI);
    // make one step older
    regPosM[1] = regPosM[0];
  }
  // limit I
  if (regul_pos_i_limit > 1e-5)
  { // integration is to be limited
    if (regPosUI[0] > regul_pos_i_limit)
      regPosUI[0] = regul_pos_i_limit;
    else if (regPosUI[0] < -regul_pos_i_limit)
      regPosUI[0] = -regul_pos_i_limit;
  }
  // sum P-lead/P and I term
  if (regul_pos_LeadFwd)
    regPosU[0] = regPosUD[0] + regPosUI[0];
  else
    regPosU[0] = regPosE[0] + regPosUI[0];
  //
  if (regul_pos_u_limit > 0.001)
  { // output limit enabled
    if (regPosU[0] > regul_pos_u_limit)
      regPosU[0] = regul_pos_u_limit;
    else if (regPosU[0] < -regul_pos_u_limit)
      regPosU[0] = -regul_turn_u_limit;
  }
  // make one step older
  regPosE[1] = regPosE[0];
  regPosUD[1] = regPosUD[0];
  regPosUI[1] = regPosUI[0];      
}

///////////////////////////////////////////////////////

/**
 * Do wall follow control
 * \param ref is desired position (cm)
 * \param measured is measured position (cm)
 *  */
void regulWall(float ref, float measured)
{
  if (regul_wall_LeadFwd)
  { // calculate new error
    regWallE[0] = ref - measured; // both angles
    regWallE[0] *= regul_wall_kp;
    // lead in fwd chain
    // filt 1 order ( input array, output array, x-params (numerator), y-params(denom))
    filt1order(regWallE, regWallUD, regWallParE, regWallParU);
    // integral part - output of lead is input to I
    filt1order(regWallUD, regWallUI, regWallParEI, regWallParUI);
  }
  else
  { // lead in measured path
    regWallM[0] = measured;
    // make Lead filtering
    filt1order(regWallM, regWallUD, regWallParE, regWallParU);
    // ensure also that measured and reference are in same rotation
    regWallE[0] = ref - measured;
    // then the difference should be valid also after the lead filter
    regWallE[0] = (ref - regWallUD[0]) * regul_wall_kp;
    // integral part - output of lead is input to I
    filt1order(regWallE, regWallUI, regWallParEI, regWallParUI);
    // make one step older
    regWallM[1] = regWallM[0];
  }
  // limit I
  if (regul_wall_i_limit > 1e-5)
  { // integration is to be limited
    if (regWallUI[0] > regul_wall_i_limit)
      regWallUI[0] = regul_wall_i_limit;
    else if (regWallUI[0] < -regul_wall_i_limit)
      regWallUI[0] = -regul_wall_i_limit;
  }
  // sum P-lead/P and I term
  if (regul_wall_LeadFwd)
    regWallU[0] = regWallUD[0] + regWallUI[0];
  else
    regWallU[0] = regWallE[0] + regWallUI[0];
  //
  if (regul_wall_u_limit > 0.001)
  { // output limit enabled
    if (regWallU[0] > regul_wall_u_limit)
      regWallU[0] = regul_wall_u_limit;
    else if (regWallU[0] < -regul_wall_u_limit)
      regWallU[0] = -regul_wall_u_limit;
  }
  // make one step older
  regWallE[1] = regWallE[0];
  regWallUD[1] = regWallUD[0];
  regWallUI[1] = regWallUI[0];      
}

///////////////////////////////////////////////////////

void regulBal(float ref, float measured)
{
  if (regul_bal_LeadFwd or regul_bal_Lead_gyro)
  { // lead in fwd, so first error and Kp
    regBalE[0] = (ref - measured) * regul_bal_kp;
    // then lead part filter
    if (regul_bal_Lead_gyro)
    { // Lead from gyro, we take turnrate info directly from source (gyro tilt rate)
      regBalUD[0] = regBalE[0] - regul_bal_td * gyroTiltRate * regul_bal_kp;
    }
    else
    { // normal lead
      filt1order(regBalE, regBalUD, regBalParE, regBalParU);
    }
    // integral part - output of lead is input to I
    filt1order(regBalUD, regBalUI, regBalParEI, regBalParUI);
  }
  else
  { // lead in measured branch
    regBalM[0] = measured;
    // lead part
    filt1order(regBalM, regBalUD, regBalParE, regBalParU);
    // 
    regBalE[0] = (ref - regBalUD[0]) * regul_bal_kp;
    // integral part - output of lead is input to I
    filt1order(regBalE, regBalUI, regBalParEI, regBalParUI);
    // save old M value
    regBalM[1] = regBalM[0];
  }
  // integration limit - if active
  if (regul_bal_i_limit > 1e-5)
  { // implement limit
    if (regBalUI[0] > regul_bal_i_limit)
      regBalUI[0] = regul_bal_i_limit;
    else if (regBalUI[0] < -regul_bal_i_limit)
      regBalUI[0] = -regul_bal_i_limit;
  }
  // sum P-lead/P and I term
  if (regul_bal_LeadFwd or regul_bal_Lead_gyro)
    regBalU[0] = regBalUD[0] + regBalUI[0];
  else
    regBalU[0] = regBalE[0] + regBalUI[0];
  
  if (regul_bal_u_limit > 0.001)
  { // output limit enabled
    if (regBalU[0] > regul_bal_u_limit)
      regBalU[0] = regul_bal_u_limit;
    else if (regBalU[0] < -regul_bal_u_limit)
      regBalU[0] = -regul_bal_u_limit;
  }
  // make one step older
  regBalE[1] = regBalE[0];
  regBalUD[1] = regBalUD[0];
  regBalUI[1] = regBalUI[0];      
}

///////////////////////////////////////////////////////

// void regulTiltVel(float ref, float measured)
// {
//   if (regul_tilt_LeadFwd)
//   { // lead in fwd, so first error and Kp
//     regTVE[0] = (ref - measured) * regul_tilt_kp;
//     // then lead part filter
//     filt1order(regTVE, regTVUD, regTVParE, regTVParU);
//     // integral part - output of lead is input to I
//     filt1order(regTVUD, regTVUI, regTVParEI, regTVParUI);
//   }
//   else
//   { // lead in measured branch
//     regTVM[0] = measured;
//     // lead part
//     filt1order(regTVM, regTVUD, regTVParE, regTVParU);
//     // 
//     regTVE[0] = (ref - regTVUD[0]) * regul_tilt_kp;
//     // integral part - output of lead is input to I
//     filt1order(regTVE, regTVUI, regTVParEI, regTVParUI);
//     // save old M value
//     regTVM[1] = regTVM[0];
//   }
//   // integration limit - if active
//   if (regul_tilt_i_limit > 1e-5)
//   { // implement limit
//     if (regTVUI[0] > regul_tilt_i_limit)
//       regTVUI[0] = regul_tilt_i_limit;
//     else if (regTVUI[0] < -regul_tilt_i_limit)
//       regTVUI[0] = -regul_tilt_i_limit;
//   }
//   // sum P-lead/P and I term
//   if (regul_tilt_LeadFwd)
//     regTVU[0] = regTVUD[0] + regTVUI[0];
//   else
//     regTVU[0] = regTVE[0] + regTVUI[0];
//   // output limit
//   if (regul_tilt_u_limit > 0.001)
//   { // output limit enabled
//     if (regTVU[0] > regul_tilt_u_limit)
//       regTVU[0] = regul_tilt_u_limit;
//     else if (regTVU[0] < -regul_tilt_u_limit)
//       regTVU[0] = -regul_tilt_u_limit;
//   }
//   // make one step older
//   regTVE[1] = regTVE[0];
//   regTVUD[1] = regTVUD[0];
//   regTVUI[1] = regTVUI[0];      
// }

///////////////////////////////////////////////////////

void regulBalVel(float ref, float measured)
{
  bool ulimit = false;
  if (regul_balVel_LeadFwd)
  { // lead in fwd, so first error and Kp
    regBalVelE[0] = (ref - measured) * regul_balvel_kp;
    // then lead part filter
    // if no lead, then regBalVelUD is same as regBalVelE
    filt1order(regBalVelE, regBalVelUD, regBalVelParE, regBalVelParU);
      
    // integral part - output of lead is input to I
    filt1order(regBalVelUD, regBalVelUI, regBalVelParEI, regBalVelParUI);
  }
  else
  { // lead in measured branch
    regBalVelM[0] = measured;
    // lead part
    filt1order(regBalVelM, regBalVelUD, regBalVelParE, regBalVelParU);
    // 
    regBalVelE[0] = (ref - regBalVelUD[0]) * regul_balvel_kp;
    // integral part - output of lead is input to I
    filt1order(regBalVelE, regBalVelUI, regBalVelParEI, regBalVelParUI);
    // save old M value
    regBalVelM[1] = regBalVelM[0];
  }
  // integration limit - if active
  if (regul_balvel_i_limit > 1e-5)
  { // implement limit
    if (regBalVelUI[0] > regul_balvel_i_limit)
      regBalVelUI[0] = regul_balvel_i_limit;
    else if (regBalVelUI[0] < -regul_balvel_i_limit)
      regBalVelUI[0] = -regul_balvel_i_limit;
  }
  // sum P-lead/P and I term
  if (regul_balVel_LeadFwd)
    regBalVelU[0] = regBalVelUD[0] + regBalVelUI[0];
  else
    regBalVelU[0] = regBalVelE[0] + regBalVelUI[0];
  // make one step older
  regBalVelE[1] = regBalVelE[0];
  //
  if (regul_balvel_u_limit > 1e-5)
  { // implement limit to tilt reference
    if (regBalVelU[0] > regul_balvel_u_limit)
    {
      regBalVelU[0] = regul_balvel_u_limit; // limit to about 10 deg
      ulimit = true;
    }
    else if (regBalVelU[0] < -regul_balvel_u_limit)
    {
      regBalVelU[0] = -regul_balvel_u_limit;
      ulimit = true;
    }
  }
  regBalVelUD[1] = regBalVelUD[0];
  if (not ulimit)  
    // allow integration
    regBalVelUI[1] = regBalVelUI[0];      
}

/**
 * to control as a P, P-Lead with an optional I-term
 * \param e is error and historic error
 * \param ud is the storage for the result of the P or P-Lead part of controller (size 2 floats)
 * \param ui is the storage for the result of the integral part of controller (size 2 floats)
 * \param u is the output of the controller u[0] (and storage for the last controller value 
 * \param i_limit is maximum integration value, if 0.0 (or negative) then no limit 
 * \param par is index to controller parameters: 0:velocity, 1: turn, 2: balance, 3: balance velocity */
void regulatorPILead(float * e, float * ud, float * ui, float * u, float i_limit, int par)
{
  static int a = 1000;
  switch (par)
  {
    case 0: // motor velocity controller
//       if (regul_vel_LeadFwd)
//       {
//         e[0] = ref - measured[0];
//         ud[0] = filt1order(e, ud, regVelParE, regVelParU);
// //         ud[0] = -regVelParU[1] * ud[1] + 
// //                 (regVelParE[0] * e[0] + 
// //                 regVelParE[1] * e[1]);
//       }
//       else
//       {
//         e[0] = -regVelParU[1] * ud[1] + 
//                 (regVelParE[0] * measured[0] + 
//                  regVelParE[1] * measured[1]);
//         ikke rigtigt godt
//         skal nok have funktion for 1 transfer function med in og out som array
//       }
//       //
//       // integrator used Kp-Lead result 'u' as input
//       ui[0] = -regVelParUI[1] * ui[1] + 
//               (regVelParEI[0] * ud[0] + 
//                regVelParEI[1] * ud[1]);
      break;
    case  1: // turn controller
      ud[0] = -regTurnParU[1] * ud[1] + 
              (regTurnParE[0] * e[0] + 
               regTurnParE[1] * e[1]);
      //
      // integrator used Kp-Lead result 'u' as input
      ui[0] = -regTurnParUI[1] * ui[1] + 
              (regTurnParEI[0] * ud[0] + 
               regTurnParEI[1] * ud[1]);
      break;
    case  2: // balance controller
      ud[0] = -regBalParU[1] * ud[1] + 
              (regBalParE[0] * e[0] + 
               regBalParE[1] * e[1]);
      //
      // integrator used Kp-Lead result 'u[0]' as input
      ui[0] = -regBalParUI[1] * ui[1] + 
              (regBalParEI[0] * ud[0] + 
               regBalParEI[1] * ud[1]);
      // debug
      if (a < 10 and hbTimerCnt % 100 < 5)
      { // balance control fails if this "if" is missing !!!!!
        // I fail to see why.
        const int MSL = 200;
        char s[MSL];
        snprintf(s, MSL, "# debug bal ctrl: hb %.3f e[0]=%.2f, ud[0]=%5.2f, ui[0]=%5.2f \r\n", time, e[0],  ud[0], ui[0]);
        usb_send_str(s);
      }
      // debug end
      break;
    case  3: // velocity control when balancing - optional use complex poles in lead
      ud[0] = -regBalVelParU[1] * ud[1] -
               regBalVelParU[2] * ud[2] + 
               regBalVelParE[0] * e[0] + 
               regBalVelParE[1] * e[1] + 
               regBalVelParE[2] * e[2];
      // limit control - before integrator
      if (i_limit > 0.0)
      { // limit to less than integrator
        const float pLimitFac = 0.3;
        if (ud[0] > i_limit * pLimitFac)
          ud[0] = i_limit * pLimitFac;
        else if (ud[0] < -i_limit * pLimitFac)
          ud[0] = -i_limit * pLimitFac;
      }
      // integrator used Kp-Lead result 'u' as input
      ui[0] = -regBalVelParUI[1] * ui[1] + 
              (regBalVelParEI[0] * ud[0] + 
               regBalVelParEI[1] * ud[1]);
      // remember extra term
      ud[2] = ud[1];
      e[2] = e[1];
      break;
    default:
      break;
  }      
  if (i_limit > 0.0)
  { // limit possible integration term
    if (ui[0] > i_limit)
      ui[0] = i_limit;
    else if (ui[0] < -i_limit)
      ui[0] = -i_limit;
  }
  // add Kp-Lead term with I-term
  u[0] = ud[0] + ui[0];
  // save for next iteration
  e[1] = e[0];
  ud[1] = ud[0];
  ui[1] = ui[0];
}

///////////////////////////////////////////////////

void regulatorVelBoth()
{ // make speed control of both motors
  if (missionState > 1)
  { // we are in a mission
    float velRef, balURef;
    if (balance_active and mission > 1)
    { // do not use mission velocity, comes from balance control
      velRef = 0.0;
      // no acceleration limit on balance control
      balURef = regul_bal_uvel;
    }
    else
    { // use mission velocity as input - before acc limit
      velRef = mission_vel_ref + mission_wall_vel_ref;
      balURef = 0.0;
    }
    if (regul_vel_use)
    { // implement velocity control on motor voltage
      // acceleration control is here (if requested)
      float accSampleLimit = regul_acc_limit * SAMPLETIME;
      if (regul_acc_limit < 98 and regul_acc_limit >= 0.01)
      { // usable acceleration limit
        // left wheel
        float diff = velRef - regul_turn_vel_reduc[0] - regul_vel_ref[0];
        if (diff > accSampleLimit)
          regul_vel_ref[0] += accSampleLimit;
        else if (diff < -accSampleLimit)
          regul_vel_ref[0] -= accSampleLimit;
        else
          regul_vel_ref[0] = velRef - regul_turn_vel_reduc[0];
        // right wheel
        diff = velRef - regul_turn_vel_reduc[1] - regul_vel_ref[1];
        if (diff > accSampleLimit)
          regul_vel_ref[1] += accSampleLimit;
        else if (diff < -accSampleLimit)
          regul_vel_ref[1] -= accSampleLimit;
        else
          regul_vel_ref[1] = velRef - regul_turn_vel_reduc[1];
      }
      else
      { // no acceleration limit requested
        regul_vel_ref[0] = velRef - regul_turn_vel_reduc[0];
        regul_vel_ref[1] = velRef - regul_turn_vel_reduc[1];
      }
      // add balance value without acceleration limit
      regul_vel_tot_ref[0] = regul_vel_ref[0] + balURef;
      regul_vel_tot_ref[1] = regul_vel_ref[1] + balURef;
      // calculate regulator error
      regulWheelVel(regul_vel_tot_ref[0], wheelVelocityEst[0], true);  // left
      regulWheelVel(regul_vel_tot_ref[1], wheelVelocityEst[1], false); // right
      // transefer to motor voltage
      motorAnkerVoltage[0] = regVelULeft[0];
      motorAnkerVoltage[1] = regVelURight[0];
    }
    else
    { // store also in regul_vel_tot_ref for proper logging
      // no acc limit
      regul_vel_tot_ref[0] = velRef + balURef - regul_turn_vel_reduc[0];
      regul_vel_tot_ref[1] = velRef + balURef - regul_turn_vel_reduc[1];
      // and then direct to motor voltage
      motorAnkerVoltage[0] = regul_vel_tot_ref[0];
      motorAnkerVoltage[1] = regul_vel_tot_ref[1];
    }
  }
  else if (regVelELeft[0] != 0.0)
  { // clean up after last mission
    regVelELeft[1] = 0;
//    regVelELeft[2] = 0;
    regVelERight[1] = 0;
//    regVelERight[2] = 0;
    regVelUDLeft[1] = 0;
    regVelUDRight[1] = 0;
    regVelUILeft[1] = 0;
    regVelUIRight[1] = 0;
    //    regVelERight[2] = 0;
    motorAnkerVoltage[0] = 0;
    motorAnkerVoltage[1] = 0;
//     regul_vel_ref[0] = 0.0;
//     regul_vel_ref[1] = 0.0;
//     regul_turn_vel_reduc[0] = 0;
//     regul_turn_vel_reduc[1] = 0;
    motorSetEnable(false, false);
//     zeroVelEstimator();
  }
}

/**
 * position controller  */
void regulator_position()
{ // turn regulator - using pose heading and main mission turn reference
  if (missionState > 0)
  { // we are running and mission, 
    // this controller follows line
    if (missionState > 1)
      regulPosition(mission_pos_ref, distance - misStartDist);
    else
      // run straight until line - or something
      regPosU[0] = 0;
    // output as new velocity reference
    mission_vel_ref = regPosU[0];
  }
  else
  { // no mission active - reset controller
    mission_pos_ref = 0.0;
    regPosE[1] = 0;
    regPosUD[1] = 0;
    regPosUI[1] = 0;
    mission_vel_ref = 0;
  }
}

/**
 * position controller  */
void regulator_wall()
{ // turn regulator - using pose heading and main mission turn reference
  if (missionState > 0)
  { // we are running and mission, 
    // this controller follows line
    float m;
    if (mission_wall_turn)
    { // brug kortest afstand
      if (irDistance[0]  < irDistance[1] * 0.7)
        m = irDistance[0];
      else
        m = irDistance[1] * 0.65;
    }
    else
      // sensor looking forward, 
      m = irDistance[1];
    if (missionState > 1)
    {
      regulWall(mission_wall_ref, m);
      if (m > 0.5)
        // too far away to trust for control
        regWallU[0] = 0;
    }
    else
      // run straight until line - or something
      regWallU[0] = 0;
    //
    if (mission_wall_turn)
    { // we are controlling turn
      if (regWallU[0] > 0.0)
      { // reduce inner wheel only
        regul_turn_vel_reduc[0] = regWallU[0];
        regul_turn_vel_reduc[1] = 0;
      }
      else
      {
        regul_turn_vel_reduc[0] = 0;
        regul_turn_vel_reduc[1] = -regWallU[0];
      }
      mission_wall_vel_ref = 0;
    }
    else 
    { // 
      mission_vel_ref = 0;
      mission_wall_vel_ref = -regWallU[0];
    }
  }
  else
  { // no mission active - reset controller
    mission_wall_ref = 0.0;
    regWallE[1] = 0;
    regWallUD[1] = 0;
    regWallUI[1] = 0;
    if (regul_wall_do_turn)
    {
      regul_turn_vel_reduc[0] = 0.0;
      regul_turn_vel_reduc[1] = 0.0;
    }
    else
      mission_wall_vel_ref = 0;
  }
}



bool flag = true;
/**
 * turn controller */
void regulator_turn()
{ // turn regulator - using pose heading and main mission turn reference
  if (missionState > 0)
  { // we are running and mission, 
    if (mission_turn_do)
    { // handle turns by difference in wheel speed only
      if (mission_tr_turn > 0)
      { // regulate left wheel (relative to right wheel)
        float vel = wheelVelocityEst[1];
        regul_turn_vel_reduc[0] = vel /*mission_vel_ref*/ /*_preturn*/ * odoWheelBase / 
                             (mission_turn_radius + odoWheelBase / 2.0);
        regul_turn_vel_reduc[1] = 0;
      }
      else
      { // regulate right wheel
        float vel = wheelVelocityEst[0];
        regul_turn_vel_reduc[1] = vel /*mission_vel_ref*/ /*_preturn*/ * odoWheelBase / 
                             (mission_turn_radius + odoWheelBase / 2.0);
        regul_turn_vel_reduc[0] = 0;
      }
      if (flag)
      {
        const int MSL = 50;
        char s[MSL];
        snprintf(s, MSL, "# turn reduc %.3f %.2f %.2f\r\n", time, regul_turn_vel_reduc[0], regul_turn_vel_reduc[1]);
        usb_send_str(s);
        flag = false;
      }
    }
    else
    { // this controller keeps heading during straight path only
      flag = true;
      if (regul_turn_use)
      { // turn controller effective
        regulSteer(mission_turn_ref, pose[2]);
        regul_turn_vel_reduc[0] =  regTurnU[0];
        regul_turn_vel_reduc[1] = -regTurnU[0];
      }
      else
      {  // open loop control
        if (mission == 1)
        { // turn step mission in open loop
          regul_turn_vel_reduc[0]  = mission_turn_ref;
          regul_turn_vel_reduc[1]  = -mission_turn_ref;
        }
        else
        { // user mission in open loop - when no turn
          regul_turn_vel_reduc[0]  = 0; // mission_turn_ref;
          regul_turn_vel_reduc[1]  = 0; // -mission_turn_ref;
        }
      }
    }
  }
  else
  { // no mission active - reset controller
    mission_turn_ref = 0.0;
    regTurnE[1] = 0;
    regTurnUD[1] = 0;
    regTurnUI[1] = 0;
//     regul_turn_vel_reduc[0] = 0.0;
//     regul_turn_vel_reduc[1] = 0.0;
  }
}

/**
 * turn controller with line follow */
void regulator_line()
{ // turn regulator - using pose heading and main mission turn reference
  if (missionState > 0)
  { // we are running and mission, 
    // this controller follows line
    if (mission_line_LeftEdge and lsLeftValid)
        regulLine(mission_line_ref, lsLeftSide);
    else if (not mission_line_LeftEdge and lsRightValid)
        regulLine(mission_line_ref, lsRightSide);
    else
      // run straight until line - or something
      regLineU[0] = 0;
    if (regLineU[0] > 0.0)
    { // reduce inner wheel only
      regul_turn_vel_reduc[0] = regLineU[0];
      regul_turn_vel_reduc[1] = 0;
    }
    else
    {
      regul_turn_vel_reduc[0] = 0;
      regul_turn_vel_reduc[1] = -regLineU[0];
    }
  }
  else
  { // no mission active - reset controller
    mission_turn_ref = 0.0;
    regLineE[1] = 0;
    regLineUD[1] = 0;
    regLineUI[1] = 0;
//     regul_turn_vel_reduc[0] = 0.0;
//     regul_turn_vel_reduc[1] = 0.0;
  }
}

/**
 * turn controller for wall follow */
// void regulator_wall()
// { // turn regulator - using one of IR distance sensors
//   if (missionState > 0)
//   { // we are running and mission, 
//     // this controller follows line
//     if (mission_wall_LeftEdge)
//       regulLine(mission_wall_ref, irDistance[0]);
//     else if (not mission_wall_LeftEdge)
//       regulLine(mission_line_ref, irDistance[1]);
//     //
//     regul_turn_vel_reduc[0] = regWallU[0];
//     regul_turn_vel_reduc[1] = -regWallU[0];
//   }
//   else
//   { // no mission active - reset controller
//     mission_wall_ref = 0.0;
//     regWallE[1] = 0;
//     regWallUD[1] = 0;
//     regWallUI[1] = 0;
//     regul_turn_vel_reduc[0] = 0.0;
//     regul_turn_vel_reduc[1] = 0.0;
//   }
// }


/**
 * balance and in-ballance velocity controller management
 * velocity is acceleration limited in the in-ballance velocity controller.
 */
void regulator_naa_ss_bal()
{
  if (missionState <2) {} 
  if (missionState > 1)
  { // Nils original implementation
//     intgbal += 0.00125*(
//       (pose[3]+regul_bal_i_limit-1.0)*regul_bal_kp +
//       gyroTilt*regul_bal_ti + 
//       wheelVelocity[0]*regul_bal_td +
//       (wheelPosition[0]-posoff)*regul_bal_alpha
//     );
    intgbal += SAMPLETIME *(
              pose[3]*regul_ss_k_tilt +
              gyroTiltRate*regul_ss_k_gyro + 
              ((wheelVelocityEst[0] + wheelVelocityEst[1])/ 2.0 - mission_vel_ref) * regul_ss_k_vel +
              ((wheelPosition[0] + wheelPosition[1])/2.0 - posoff - mission_pos_ref) * regul_ss_k_pos +
              (regul_bal_uvel - mission_vel_ref_ref) * regul_ss_k_motor
                       );
    regul_bal_uvel = intgbal;
  }
  else
  {
    intgbal = 0.0;
    posoff = (wheelPosition[0] + wheelPosition[1]) / 2.0;
    regBalE[1] = 0.0;
    regBalUD[1] = 0.0;
    regBalUI[1] = 0.0;
    regBalVelE[1] = 0.0;
    regBalVelUI[1] = 0.0;
    regul_balvel_reduc = (wheelVelocityEst[0] + wheelVelocityEst[1])/ 2.0;
    regul_bal_uvel = 0.0;
  }  
}


/**
 * balance and in-ballance velocity controller management
 * velocity is acceleration limited in the in-ballance velocity controller.
 */
void regulator_bal()
{    
  if (missionState > 1)
  { // we are running
    // 
    if (regul_balvel_use)
    { // acceleration limit
      // use velocity difference between desired and last commanded velocity
      float accSampleLimit = regul_acc_limit * SAMPLETIME;
      float diff = mission_vel_ref + mission_wall_vel_ref - regul_balvel_reduc;
      if (regul_acc_limit > 0.001 and regul_acc_limit < 90.0)
      {
        if (diff > accSampleLimit)
          regul_balvel_reduc += accSampleLimit;
        else if (diff < -accSampleLimit)
          regul_balvel_reduc -= accSampleLimit;
        else
          regul_balvel_reduc = mission_vel_ref + mission_wall_vel_ref;
      }
      else
        regul_balvel_reduc = mission_vel_ref + mission_wall_vel_ref;
      // // calculate control value
      regulBalVel(regul_balvel_reduc, (wheelVelocityEst[0] + wheelVelocityEst[1])/ 2.0);
      // use result as ref for tilt control
//       if (regul_balvel_u_limit > 1e-5)
//       { // implement limit to tilt reference
//         if (regBalVelU[0] > regul_balvel_u_limit)
//           balTiltRef = regul_balvel_u_limit; // limit to about 10 deg
//         else if (regBalVelU[0] < -regul_balvel_u_limit)
//           balTiltRef = -regul_balvel_u_limit;
//         else
//           balTiltRef = regBalVelU[0];
//       }
//       else
      balTiltRef = regBalVelU[0];        
    }
    else
    { // no velocity regulator, so use step as angle reference
      balTiltRef = mission_vel_ref;
    }
    // 
    if (regul_bal_use)
    { // balance control (balance tilt offset should be (almost) zero for top-mounted IMU)
      regulBal(balTiltRef, pose[3]);
      // controls tilt velocity - in rad/c on motor axle
      regul_bal_uvel =  regBalU[0];
    }
    else
    { // pass reference to next controller
      regul_bal_uvel = balTiltRef;
    }
  }
  else
  { // make sure all old values are zero
    regBalE[1] = 0.0;
    regBalUD[1] = 0.0;
    regBalUI[1] = 0.0;
    regBalVelE[1] = 0.0;
    regBalVelUI[1] = 0.0;
    regul_balvel_reduc = (wheelVelocityEst[0] + wheelVelocityEst[1])/ 2.0;
    regul_bal_uvel = 0.0;
  }
}

/**
 * This function is called every time it is time to do a new control calculation
 * weather in a mission or not
 * The new motor values are implemented once this functon returns. 
 * */ 
void controlTick(void)
{ // NB! must return fast, i.e. no long processing here, except
  // after end of mission (default state)
  // usb_serial_write(".", 1);
  sendMissionStatusChanged(false);
//   if (missionStateLast != missionState or misLineNum != misLineNumLast)
//   {
//     sendMissionStatusChanged();
//     missionStateLast = missionState;
//     misLineNumLast = misLineNum;
// //     if (missionState == 1)
// //     { // we just started - no control yet
// //       // reset acceleration limit
// //       const int MSL = 80;
// //       char s[MSL];
// //       snprintf(s, MSL, "# mission state 1 velreduc %g %g\n", regul_vel_ref[0], regul_vel_ref[1]);
// //       usb_send_str(s);
// //     }
//     usb_send_str("# not here\n");
//   }
//   if (missionState > 0 and missionStop)
//     // switch to default state - to stop mission
//     missionState = -1;
  switch (mission)
  {
    case 0: // velocity step
      mission_gui_vel_step(); 
      regulatorVelBoth();
      break;
    case 1: // turn step
      mission_gui_turn_step(); 
      regulator_turn();
      regulatorVelBoth();
      break;
    case 2: // ballance step
      // ballance step using PID controller
      balance_test();
      // use PID regulator
      regulator_bal();
      regulator_turn();
      regulatorVelBoth();
      break;
    case 3: // user defined mission
      // Mission progress test, advance and implement
      user_mission();
      if (not missionStop)
      { // ready to do control loops
        if (mission_wall_use and not mission_wall_turn)
          // front distance is controlling velocity reference
          // balance or not
          regulator_wall();
        else if (mission_pos_use)
          // position regulator provides velocity reference
          regulator_position();
        // should balance be active
        if (balance_active)
          // do balance control to control velocity reference
          regulator_bal();
        // turn control - either follow edge, wall or just turn
        if (regul_line_use)
          regulator_line();
        else if (mission_wall_use and mission_wall_turn)
          regulator_wall();
        else
          regulator_turn();
        // implement desired velocity for wheels
        regulatorVelBoth();
      }
      break;
    case 4: // balance state-space step
      // ballance step using state space
      balance_ss_test();
      // use state space balance regulator
      regulator_naa_ss_bal();
      // use PID turn regulator
      if (false)
        regulator_line();
      else
        regulator_turn();
      // use wheel speed controller
      regulatorVelBoth();
      break;
    case 5: // line follow step
      mission_gui_line_step(); 
      regulator_line();
      regulatorVelBoth();
      break;
    case 6: // wall follow (step)
      //usb_send_str("# wall control not tested yet\r\n");
      mission_gui_wall_step(); 
      regulator_wall();
      regulatorVelBoth();
      break;
    case 7: // position control
      //usb_send_str("# position control not tested yet\r\n");
      mission_gui_pos_step(); 
      regulator_position();
      regulator_turn();
      regulatorVelBoth();
      break;
    case 10: // default
    case 11: // follow wall
    case 12: // balance step
    case 13: // empty
    case 14: // empty
    case 15: // empty
      // first non-default hard coded mission
      mission_hard_coded();
      break;
    default:
      mission = 0;
      break;
  } 
  if (missionStop and missionState > 0)
  { // here once, when mission (or GUI) sets stop-flag
    motorAnkerVoltage[0] = 0;
    motorAnkerVoltage[1] = 0;
    mission_turn_do = false;
    mission_vel_ref = 0;
    mission_wall_vel_ref = 0;
    mission_wall_ref = 0;
    mission_line_ref = 0;
    mission_pos_ref = 0;
    motorSetEnable(false, false);
    stopLogging();
    //misLine = NULL;
    missionLineNum = 0;
    missionStart = false;
    missionStop = false;
    missionState = 0;
    regul_balvel_reduc = 0;
    regul_turn_vel_reduc[0] = 0;
    regul_turn_vel_reduc[1] = 0;
    regul_vel_ref[0] = 0.0;
    regul_vel_ref[1] = 0.0;
    if (backToNormal)
    {
      eeConfig.eePromLoadStatus(false);
      backToNormal = false;
    }
    usb_send_str("#** Mission stopped\r\n");
  }  
}

///////////////////////////////////////////////////////

void initRegulators()
{
  // Init z-1 values for velocity
  regVelELeft[1] = 0;
  regVelERight[1] = 0;
  regVelUDLeft[1] = 0;
  regVelUDRight[1] = 0;
  // integral controller
  regVelUILeft[1] = 0;
  regVelUIRight[1] = 0;
  //
  // Init z-1 values for turn
  regPosE[1] = 0;
  regPosUD[1] = 0;
  regPosUI[1] = 0;
  // Init z-1 values for turn
  regTurnE[1] = 0;
  regTurnUD[1] = 0;
  regTurnUI[1] = 0;
  // Init z-1 values for turn
  regLineE[1] = 0;
  regLineUD[1] = 0;
  regLineUI[1] = 0;
  // Init z-1 values for wall follow
  regWallE[1] = 0;
  regWallUD[1] = 0;
  regWallUI[1] = 0;
  // Init z-1 values for bal
  regBalE[1] = 0;
  regBalUD[1] = 0;
  regBalUI[1] = 0;
  // Init z-1 values for bal
  regBalVelE[1] = 0;
  regBalVelE[2] = 0;
  regBalVelUD[1] = 0;
  regBalVelUD[2] = 0;
  regBalVelUI[1] = 0; 
  //
  regTVE[1] = 0;
  regTVUD[1] = 0;
  regTVUI[1] = 0;
  // velocity regulator init
  regulatorInitComplexLead(1.0, regul_vel_ti, regul_vel_td, regul_vel_alpha, 0.0,
                 regVelParU, regVelParE, regVelParUI, regVelParEI);
  // position regulator init
  regulatorInitComplexLead(1.0, regul_pos_ti, regul_pos_td, regul_pos_alpha, 0.0,
                           regPosParU, regPosParE, regPosParUI, regPosParEI);
  // turn regulator
  regulatorInitComplexLead(1.0, regul_turn_ti, regul_turn_td, regul_turn_alpha, 0.0,
                 regTurnParU, regTurnParE, regTurnParUI, regTurnParEI);
  // line follow regulator
  regulatorInitComplexLead(1.0, regul_line_ti, regul_line_td, regul_line_alpha, 0.0,
                           regLineParU, regLineParE, regLineParUI, regLineParEI);
  // wall follow regulator
  regulatorInitComplexLead(1.0, regul_wall_ti, regul_wall_td, regul_wall_alpha, 0.0,
                           regWallParU, regWallParE, regWallParUI, regWallParEI);
  // balance tilt velocity regulator init
//   regulatorInitComplexLead(1.0, regul_tilt_ti, regul_tilt_td, regul_tilt_alpha, 0.0,
//                  regTVParU, regTVParE, regTVParUI, regTVParEI);
  // balance regulator init
  regulatorInitComplexLead(1.0, regul_bal_ti, regul_bal_td, regul_bal_alpha, 0.0,
                           regBalParU, regBalParE, regBalParUI, regBalParEI);
  // balance regulator init
  // with (possible) complex pole pair in lead
  if (false)
    // 2nd order Lead
    // has a bug - do not use
    regulatorInitComplexLead(1.0, regul_balvel_ti, regul_balvel_td, regul_balvel_alpha, regul_balvel_zeta, 
                 regBalVelParU, regBalVelParE, regBalVelParUI, regBalVelParEI);
  else
    // 1st order Lead
    regulatorInitComplexLead(1.0, regul_balvel_ti, regul_balvel_td, regul_balvel_alpha, 0.0, 
                           regBalVelParU, regBalVelParE, regBalVelParUI, regBalVelParEI);
}

//////////////////////////////////////////////////

bool setRegulator(const char * line)
{
  bool used = false;
  if (line[2] == '=' and line[0] == 'r')
  { // one character assignment command
    char * p1 = (char *)(&line[3]);
    if (*p1 >= ' ')
    {
      used = true;
      switch (line[1])
      { // rt= use kp td alpha
        case 't':
          regul_turn_use = strtol(p1, &p1, 10);
          if (*p1 >= ' ')
            regul_turn_kp = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_turn_ti = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_turn_td = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_turn_alpha = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_turn_i_limit = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_turn_step_time_on = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_turn_step_time_off = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_turn_step_val = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_turn_step_vel = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_turn_LeadFwd = strtol(p1, &p1, 10);
          if (*p1 >= ' ')
            regul_turn_u_limit = strtof(p1, &p1);
          break;
        case 'l': // line follow
          regul_line_use = strtol(p1, &p1, 10);
          regul_line_use = true;
          if (*p1 >= ' ')
            regul_line_kp = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_line_ti = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_line_td = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_line_alpha = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_line_i_limit = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_line_step_time_on = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_line_step_from = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_line_step_to = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_line_step_vel = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_line_LeadFwd = strtol(p1, &p1, 10);
          if (*p1 >= ' ')
            regul_line_u_limit = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_line_followLeft = strtol(p1, &p1, 10);
          break;
        case 'd': // wall follow
          regul_wall_use = strtol(p1, &p1, 10);
          if (*p1 >= ' ')
            regul_wall_kp = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_wall_ti = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_wall_td = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_wall_alpha = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_wall_i_limit = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_wall_step_time = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_wall_step_from = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_wall_step_to = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_wall_step_vel = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_wall_LeadFwd = strtol(p1, &p1, 10);
          if (*p1 >= ' ')
            regul_wall_u_limit = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_wall_use = strtol(p1, &p1, 10);
          if (*p1 >= ' ')
            regul_wall_sensor1 = strtol(p1, &p1, 10);
          break;
        case 'v':
          regul_vel_use = strtol(p1, &p1, 10);
          if (*p1 >= ' ')
            regul_vel_kp = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_vel_ti = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_vel_td = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_vel_alpha = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_vel_i_limit = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_vel_step_time = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_vel_step_from = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_vel_step_to = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_acc_limit = strtof(p1, &p1);
          if (*p1 >= ' ')
            // use estimated velocity (unused)
            regul_vel_LeadFwd = strtol(p1, &p1, 10);
          if (*p1 >= ' ')
            // use estimated velocity
            regul_vel_LeadFwd = strtol(p1, &p1, 10);
          if (*p1 >= ' ')
            // use estimated velocity
            max_motor_voltage = strtof(p1, &p1);
          // void regulatorInit(float kp, float ti, float td, float al, float * zu, float * ze)
          break;
        case 'p': // position controller
          regul_pos_use = strtol(p1, &p1, 10);
          if (*p1 >= ' ')
            regul_pos_kp = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_pos_ti = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_pos_td = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_pos_alpha = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_pos_i_limit = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_pos_step_time = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_pos_step_from = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_pos_step_to = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_pos_LeadFwd = strtol(p1, &p1, 10);
          if (*p1 >= ' ')
            regul_pos_u_limit = strtof(p1, &p1);
          // void regulatorInit(float kp, float ti, float td, float al, float * zu, float * ze)
          break;
        case 's':
          // (gyro) tilt velocity parameters
          regul_ss_use = strtol(p1, &p1, 10);
          if (*p1 >= ' ')
            regul_ss_k_tilt = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_ss_k_gyro = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_ss_k_pos = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_ss_k_vel = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_ss_k_motor = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_ss_u_limit = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_ss_step_time = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_ss_step_from = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_ss_step_to = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_ss_step_pos = strtol(p1, &p1, 10);
          break;
        case 'b':
          // balance control parameters
          regul_bal_use = strtol(p1, &p1, 10);
          if (*p1 >= ' ')
            regul_bal_kp = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_bal_ti = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_bal_td = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_bal_alpha = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_bal_i_limit = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_balvel_step_time = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_balvel_step_from = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_balvel_step_to = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_bal_LeadFwd = strtol(p1, &p1, 10);
          if (*p1 >= ' ')
            regul_bal_u_limit = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_bal_Lead_gyro = strtol(p1, &p1, 10);
          break;
        case 'm':
          regul_balvel_use = strtol(p1, &p1, 10);
          if (*p1 >= ' ')
            regul_balvel_kp = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_balvel_ti = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_balvel_td = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_balvel_alpha = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_balvel_zeta = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_balvel_i_limit = strtof(p1, &p1);
          if (*p1 >= ' ')
            regul_balVel_LeadFwd = strtol(p1, &p1, 10);
          if (*p1 >= ' ')
            regul_balvel_u_limit = strtof(p1, &p1);
          break; 
        default:
          used = false;
      }      
    }
  }
  if (used)
    // initialize regulator
    initRegulators();
  return used;
}

/**
 * save controller configuration to EE Prom */
void eePromSaveCtrl()
{ // mission
  uint8_t f;
  eeConfig.pushByte(mission);
  // vel ctrl
  eeConfig.pushByte(regul_vel_use);
  eeConfig.pushFloat(regul_vel_kp);
  eeConfig.pushFloat(regul_vel_ti);
  eeConfig.pushFloat(regul_vel_td);
  eeConfig.pushFloat(regul_vel_alpha);
  eeConfig.pushFloat(regul_vel_i_limit);
  eeConfig.pushFloat(regul_vel_step_time);
  eeConfig.pushFloat(regul_vel_step_from);
  eeConfig.pushFloat(regul_vel_step_to);
  eeConfig.pushByte(1);
  eeConfig.pushByte(regul_vel_LeadFwd);
  eeConfig.pushFloat(max_motor_voltage);
  // pos ctrl
  eeConfig.pushByte(regul_pos_use);
  eeConfig.pushFloat(regul_pos_kp);
  eeConfig.pushFloat(regul_pos_ti);
  eeConfig.pushFloat(regul_pos_td);
  eeConfig.pushFloat(regul_pos_alpha);
  eeConfig.pushFloat(regul_pos_i_limit);
  eeConfig.pushFloat(regul_pos_step_time);
  eeConfig.pushFloat(regul_pos_step_from);
  eeConfig.pushFloat(regul_pos_step_to);
  eeConfig.pushByte(regul_pos_LeadFwd);
  eeConfig.pushFloat(regul_pos_u_limit);
  // turn
  eeConfig.pushByte(regul_turn_use);
  eeConfig.pushFloat(regul_turn_kp);
  eeConfig.pushFloat(regul_turn_ti);
  eeConfig.pushFloat(regul_turn_td);
  eeConfig.pushFloat(regul_turn_alpha);
  eeConfig.pushFloat(regul_turn_i_limit);
  eeConfig.pushFloat(regul_turn_u_limit);
  eeConfig.pushFloat(regul_turn_step_time_on);
  eeConfig.pushFloat(regul_turn_step_time_off);
  eeConfig.pushFloat(regul_turn_step_val);
  eeConfig.pushFloat(regul_turn_step_vel);
  eeConfig.pushFloat(regul_acc_limit);
  eeConfig.pushByte(regul_turn_LeadFwd);
  // balance ctrl
  eeConfig.pushByte(regul_bal_use);
  eeConfig.pushFloat(regul_bal_kp);
  eeConfig.pushFloat(regul_bal_ti);
  eeConfig.pushFloat(regul_bal_td);
  eeConfig.pushFloat(regul_bal_alpha);
  eeConfig.pushFloat(regul_bal_i_limit);
  eeConfig.pushFloat(regul_bal_u_limit);
  eeConfig.pushFloat(regul_balvel_step_time);
  eeConfig.pushFloat(regul_balvel_step_from);
  eeConfig.pushFloat(regul_balvel_step_to);
  eeConfig.pushByte(regul_balvel_use);
  eeConfig.pushByte(regul_bal_LeadFwd);
  // balance state space
  eeConfig.pushByte(regul_ss_use);
  eeConfig.pushFloat(regul_ss_k_tilt);
  eeConfig.pushFloat(regul_ss_k_gyro);
  eeConfig.pushFloat(regul_ss_k_pos);
  eeConfig.pushFloat(regul_ss_k_vel);
  eeConfig.pushFloat(regul_ss_k_motor);
  eeConfig.pushFloat(regul_ss_u_limit);
  eeConfig.pushFloat(regul_ss_step_time);
  eeConfig.pushFloat(regul_ss_step_from);
  eeConfig.pushFloat(regul_ss_step_to);
  eeConfig.pushByte(regul_ss_step_pos);
  // mission balance velocity ctrl
  eeConfig.pushFloat(regul_balvel_kp);
  eeConfig.pushFloat(regul_balvel_ti);
  eeConfig.pushFloat(regul_balvel_td);
  eeConfig.pushFloat(regul_balvel_zeta);
  eeConfig.pushFloat(regul_balvel_alpha);
  eeConfig.pushFloat(regul_balvel_i_limit);
  eeConfig.pushFloat(regul_balvel_u_limit);
  eeConfig.pushByte(regul_balVel_LeadFwd);
  // line follow
  f = 0;
  if (regul_line_use)        f |= 0x01; 
  if (regul_line_LeadFwd)    f |= 0x02; 
  if (regul_line_followLeft) f |= 0x04; 
  eeConfig.pushByte(f);
  eeConfig.pushFloat(regul_line_kp);
  eeConfig.pushFloat(regul_line_ti);
  eeConfig.pushFloat(regul_line_td);
  eeConfig.pushFloat(regul_line_alpha);
  eeConfig.pushFloat(regul_line_i_limit);
  eeConfig.pushFloat(regul_line_u_limit);
  eeConfig.pushFloat(regul_line_step_time_on);
  eeConfig.pushFloat(regul_line_step_from);
  eeConfig.pushFloat(regul_line_step_to);
  eeConfig.pushFloat(regul_line_step_vel);
  // wall follow
  f = 0;
  if (regul_wall_use)     f |= 0x01; 
  if (regul_wall_LeadFwd) f |= 0x02; 
  if (regul_wall_sensor1) f |= 0x04; 
  if (regul_wall_sign)    f |= 0x08;
  if (regul_wall_do_turn) f |= 0x10;
  eeConfig.pushByte(f);
  eeConfig.pushFloat(regul_wall_kp);
  eeConfig.pushFloat(regul_wall_ti);
  eeConfig.pushFloat(regul_wall_td);
  eeConfig.pushFloat(regul_wall_alpha);
  eeConfig.pushFloat(regul_wall_i_limit);
  eeConfig.pushFloat(regul_wall_u_limit);
  eeConfig.pushFloat(regul_wall_step_time);
  eeConfig.pushFloat(regul_wall_step_from);
  eeConfig.pushFloat(regul_wall_step_to);
  eeConfig.pushFloat(regul_wall_step_vel);
}
/**
 * load controller configuration from EE Prom
 * same order as when saved
 */
void eePromLoadCtrl()
{ // mission
  uint8_t f; // flags 8 bit
  //
  mission = eeConfig.readByte();
  // vel ctrl
  regul_vel_use = eeConfig.readByte();
  regul_vel_kp = eeConfig.readFloat();
  regul_vel_ti = eeConfig.readFloat();
  regul_vel_td = eeConfig.readFloat();
  regul_vel_alpha = eeConfig.readFloat();
  regul_vel_i_limit = eeConfig.readFloat();
  regul_vel_step_time = eeConfig.readFloat();
  regul_vel_step_from = eeConfig.readFloat();
  regul_vel_step_to = eeConfig.readFloat();
  f = eeConfig.readByte();
  regul_vel_LeadFwd = eeConfig.readByte();
  max_motor_voltage = eeConfig.readFloat();
  // pos ctrl
  regul_pos_use = eeConfig.readByte();
  regul_pos_kp = eeConfig.readFloat();
  regul_pos_ti = eeConfig.readFloat();
  regul_pos_td = eeConfig.readFloat();
  regul_pos_alpha = eeConfig.readFloat();
  regul_pos_i_limit = eeConfig.readFloat();
  regul_pos_step_time = eeConfig.readFloat();
  regul_pos_step_from = eeConfig.readFloat();
  regul_pos_step_to = eeConfig.readFloat();
  regul_pos_LeadFwd = eeConfig.readByte();
  regul_pos_u_limit = eeConfig.readFloat();
  // turn
  regul_turn_use = eeConfig.readByte();
  regul_turn_kp = eeConfig.readFloat();
  regul_turn_ti = eeConfig.readFloat();
  regul_turn_td = eeConfig.readFloat();
  regul_turn_alpha = eeConfig.readFloat();
  regul_turn_i_limit = eeConfig.readFloat();
  regul_turn_u_limit = eeConfig.readFloat();
  regul_turn_step_time_on = eeConfig.readFloat();
  regul_turn_step_time_off = eeConfig.readFloat();
  regul_turn_step_val = eeConfig.readFloat();
  regul_turn_step_vel = eeConfig.readFloat();
  regul_acc_limit = eeConfig.readFloat();
  regul_turn_LeadFwd = eeConfig.readByte();
  // balance ctrl
  regul_bal_use = eeConfig.readByte();
  regul_bal_kp = eeConfig.readFloat();
  regul_bal_ti = eeConfig.readFloat();
  regul_bal_td = eeConfig.readFloat();
  regul_bal_alpha = eeConfig.readFloat();
  regul_bal_i_limit = eeConfig.readFloat();
  regul_bal_u_limit = eeConfig.readFloat();
  regul_balvel_step_time = eeConfig.readFloat();
  regul_balvel_step_from = eeConfig.readFloat();
  regul_balvel_step_to = eeConfig.readFloat();
  regul_balvel_use = eeConfig.readByte();
  regul_bal_LeadFwd = eeConfig.readByte();
  // balance state space 
  regul_ss_use = eeConfig.readByte();
  regul_ss_k_tilt = eeConfig.readFloat();
  regul_ss_k_gyro = eeConfig.readFloat();
  regul_ss_k_pos = eeConfig.readFloat();
  regul_ss_k_vel = eeConfig.readFloat();
  regul_ss_k_motor = eeConfig.readFloat();
  regul_ss_u_limit = eeConfig.readFloat();
  regul_ss_step_time = eeConfig.readFloat();
  regul_ss_step_from = eeConfig.readFloat();
  regul_ss_step_to = eeConfig.readFloat();
  regul_ss_step_pos = eeConfig.readByte();
  // balance velocity ctrl
  regul_balvel_kp = eeConfig.readFloat();
  regul_balvel_ti = eeConfig.readFloat();
  regul_balvel_td = eeConfig.readFloat();
  regul_balvel_zeta = eeConfig.readFloat();
  regul_balvel_alpha = eeConfig.readFloat();
  regul_balvel_i_limit = eeConfig.readFloat();
  regul_balvel_u_limit = eeConfig.readFloat();
  regul_balVel_LeadFwd = eeConfig.readByte();
  //
  // line follow
  f = eeConfig.readByte();
  regul_line_use = (f & 0x01) != 0; //
  regul_line_kp = eeConfig.readFloat();
  regul_line_ti = eeConfig.readFloat();
  regul_line_td = eeConfig.readFloat();
  regul_line_alpha = eeConfig.readFloat();
  regul_line_i_limit = eeConfig.readFloat();
  regul_line_u_limit = eeConfig.readFloat();
  regul_line_step_time_on = eeConfig.readFloat();
  regul_line_step_from = eeConfig.readFloat();
  regul_line_step_to = eeConfig.readFloat();
  regul_line_step_vel = eeConfig.readFloat();
  regul_line_LeadFwd = (f & 0x02) != 0; // eePromReadByte();
  regul_line_followLeft = (f & 0x04) != 0; //eePromReadByte();
  // wall follow
  f = eeConfig.readByte();
  regul_wall_use = (f & 0x01) != 0; //
  regul_wall_kp = eeConfig.readFloat();
  regul_wall_ti = eeConfig.readFloat();
  regul_wall_td = eeConfig.readFloat();
  regul_wall_alpha = eeConfig.readFloat();
  regul_wall_i_limit = eeConfig.readFloat();
  regul_wall_u_limit = eeConfig.readFloat();
  regul_wall_step_time = eeConfig.readFloat();
  regul_wall_step_from = eeConfig.readFloat();
  regul_wall_step_to = eeConfig.readFloat();
  regul_wall_step_vel = eeConfig.readFloat();
  regul_wall_LeadFwd = (f & 0x02) != 0; //eePromReadByte();
  regul_wall_sensor1 = (f & 0x04) != 0; //eePromReadByte();
  regul_wall_sign = (f & 0x08) != 0; //eePromReadByte();
  regul_wall_do_turn = (f & 0x10) != 0; // eePromReadByte();
  //
  initRegulators();
}
