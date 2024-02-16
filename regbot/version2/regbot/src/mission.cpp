/***************************************************************************
 *   Copyright (C) 2014 by DTU                             *
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
 ***************************************************************************/
#include <string.h>
#include <stdio.h>
#include "math.h"
#include "mission.h"
//#include "serial_com.h"
#include "main.h"
#include "eeconfig.h"
#include "robot.h"
#include "control.h"
#include "linesensor.h"
#include "dist_sensor.h"
#include "data_logger.h"
#include "motor_controller.h"

UMissionLine miLines[miLinesCntMax];
int miLinesCnt = 0;

char missionErrStr[missionErrStrMaxCnt];


// the main mission class
UMission userMission;


////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////

void UMissionThread::clear(uint8_t idx)
{ // give thread a number and clear the rest
  threadNr = idx;
  lineCnt = 0;
  lineStartIndex = 0;
  misLineNum = -1;
  misStartTime = 0;
  linStartAngle = 0;
  turnAngSum = 0;
  linStartDist = 0;
  turnSumLast = 0;
  theEnd = false;
}


bool UMissionThread::advanceLine ( int16_t toLabel )
{
  if (misLineNum < lineCnt)
    // default is goto next line
    misLineNum++;
  if (toLabel > 0)
  { // there is a label number, find line
    // if label is not found in this thread, then
    // continue to next line (default action)
    for (int i = 0; i < lineCnt; i++)
      if (miLines[i + lineStartIndex].label == toLabel)
      {
        misLineNum = i;
        break;
      }
  }
  // debug
//   const int MSL = 50;
//   char s[MSL];
//   snprintf(s, MSL, "# advance line to %d of %d (label=%d)\n", misLineNum, lineCnt, toLabel);
//   usb_send_str(s);
  // debug end
  if (misLineNum < lineCnt)
  { // new line is available
    currentLine = &miLines[misLineNum + lineStartIndex];
    return true;
  }
  else
    return false;
}

/////////////////////////////////////////////////////////////////

bool UMissionThread::testFinished()
{
  bool LineEnded = false;
  //usb_send_str("# Thread::testFinished -> start\n");
  if (misLineNum < lineCnt and not theEnd)
  {
    //uint16_t labelNum = 0; // 0 is a flag for no goto
    //UMissionLine * line = &miLines[misLineNum + lineStartIndex];
    gotoLabel = 0;
    if (currentLine->finished(this, &gotoLabel, &turnEndedAtEndAngle))
    { // stop any special drive scheme set by this line
//       line->postProcess(misLastAng);
//       theEnd = not advanceLine(labelNum);
//       if (not theEnd)
//       {
//         implementNewLine();
//       }
      //usb_send_str("#Line ended!");;
      LineEnded = true;
    }
//     if (gotoLabel > 0)
//     {
//       const int MSL = 50;
//       char s[MSL];
//       snprintf(s, MSL, "# in thread goto=%d\n", gotoLabel);
//       usb_send_str(s);
//     }
  }
  else
    LineEnded = true;
  return LineEnded;
}


//////////////////////////////////////////////////////////////

bool UMissionThread::moveToNextLine()
{
  if (not theEnd)
  { // finish last line
    currentLine->postProcess(linStartAngle, turnEndedAtEndAngle);
    // advance line (maybe a goto)
    theEnd = not advanceLine(gotoLabel);
    if (not theEnd)
    { // implement all parameter settings
      implementNewLine();
    }
  }
  return theEnd;
}

//////////////////////////////////////////////////////////////

void UMissionThread::resetVisits()
{
  for (int i = 0; i < lineCnt; i++)
    miLines[i + lineStartIndex].visits = 0;
}

//////////////////////////////////////////////////////////////

void UMissionThread::startMission()
{
  resetVisits();
  misLineNum = 0;
  theEnd = false;
  if (misLineNum < lineCnt)
    implementNewLine();
  else
    theEnd = true;
}

//////////////////////////////////////////////////////////////

void UMissionThread::implementNewLine()
{
//   // debug
//   const int MSL = 120;
//   char s[MSL];
//   // debug end
 // initialize new mission line
  currentLine = &miLines[misLineNum + lineStartIndex];
  currentLine->visits++;
  // if old line controlled angle, then
  // use end angle as new reference
  if (mission_wall_turn or regul_line_use)
  { // Wall follow or line follow
    // just use current heading as new reference
    mission_turn_ref = pose[2];
    regTurnM[1] = pose[2];
  }
  if (misLineNum < miLinesCnt)
  { // prepare next line
    turnAngSum = 0; 
    turnSumLast = mission_turn_ref;
    linStartAngle = mission_turn_ref;
    turnEndedAtEndAngle = false;
    linStartDist = distance;
    misStartTime = hbTimerCnt;
    // all other settings
    currentLine->implementLine();
  }
  // every time a thread implements a new line 
  // this data is set - to allow user state reporting
  misLine = currentLine;
  misThread = threadNr;
  missionLineNum = misLineNum;
//   // debug
//   snprintf(s, MSL, "# implemented line %d in thread %d\r\n", misLineNum, threadNr);
//   usb_send_str(s);
//   // debug end
}

////////////////////////////////////////////////////////

bool UMissionThread::addLine(const char * lineToAdd, int16_t * newThreadNumber)
{
  int idx = lineCnt + lineStartIndex;
  bool isOK = idx < miLinesCntMax;
  const int MSL = 150;
  char s[MSL];
  if (isOK)
  {  
//     // debug
//     snprintf(s, MSL, "# thread %d line %d is: %s\n\r", threadNr, lineCnt, lineToAdd);
//     usb_send_str(s);
//     // debug end
    isOK = miLines[idx].decodeLine(lineToAdd, newThreadNumber);
    if (not isOK)
    { // report error
      snprintf(s, MSL, "#syntax error thread %d line %d: %s\n", threadNr, lineCnt, missionErrStr);
      usb_send_str(s);
    }
//     // debug
//     snprintf(s, MSL, "# thread %d newThread %d isOK=%d\n\r", threadNr, *newThreadNumber, isOK);
//     usb_send_str(s);
//     // debug end
    // changing thread number is handled at UMission level
    if (*newThreadNumber != threadNr)
    { // there is a new thread number
      if (lineCnt == 0)
        // but OK, as there is no lines
        threadNr = *newThreadNumber;
      else
        isOK = false;
    }
    if (isOK)
    { // OK to add
      isOK = miLines[idx].valid;
      if (isOK)
      {
        int n = miLines[idx].toString(s, MSL, false);
        if (n > 2)
        { // not an empty line (or thread keyword only)
          lineCnt++;
          if (idx >= miLinesCnt)
            miLinesCnt++;
//           // debug
//           snprintf(s, MSL, "# thread %d line increased to %d\n\r", threadNr, lineCnt);
//           usb_send_str(s);
//           // debug end
        }
      }
      else
      {
        snprintf(s, MSL, "\r\n# add line %d failed: %s\r\n", idx, missionErrStr);
        usb_send_str(s);
      }
    }
//     // debug
//     snprintf(s, MSL, "# thread %d line %d added OK =%d\n\r", threadNr, lineCnt, isOK);
//     usb_send_str(s);
//     // debug end
  }
  return isOK;
}


bool UMissionThread::modLine(int16_t line, const char * p2)
{
  bool isOK = line <= lineCnt  and line > 0;
  if (isOK)
  { // decode the line twice first for syntax check, then for real
    UMissionLine tmp;
    int16_t thnr = threadNr;
    isOK = tmp.decodeLine(p2, &thnr);
    if (thnr != threadNr)
    { // there must not be a line with new thread number
      isOK = false;
      usb_send_str("# not legal to modify thread number\n");
    }
    // debug
//     const int MSL = 50;
//     char s[MSL];
//     snprintf(s, MSL, "# line %d found isOK=%d\n", line, isOK);
//     usb_send_str(s);
    // debug end
    
    if (isOK)
    { // OK, modify the line
      int idx = line + lineStartIndex - 1;
      isOK = miLines[idx].decodeLine(p2, &thnr);
    }
  }
  else
    usb_send_str("# modify failed - no such line\n");
  return isOK;
}

/////////////////////////////////////

void UMissionThread::getLines()
{
  const int MRL = 100;
  char reply[MRL];
  snprintf(reply, MRL, "<m thread=%d\r\n", threadNr);
  usb_send_str(reply);
  for (int i = 0; i < lineCnt; i++)
  {
    int n = miLines[i + lineStartIndex].toString(reply, MRL);
    if (n > 2)
      // a thread line may have no other attributes
      usb_send_str(reply);
  }
}

/////////////////////////////////////////////////////

void UMissionThread::getToken()
{ // get token lines to console for debug
  const int MRL = 30;
  char reply[MRL];
  const int MSL = 100;
  char s[MSL];
  // first the thread number
  toTokenString(reply, MRL);
  snprintf(s, MSL, "#thread %d (%d) %s\r", threadNr, strlen(reply), reply);
  usb_send_str(s);
  // then the lines in the thread
  for (int i = 0; i < lineCnt; i++)
  {
    //usb_write("# line\n");
    miLines[i + lineStartIndex].toTokenString(reply, MRL);
    //usb_send_str(reply);
    snprintf(s, MSL, "#line %d (%d) %s\r", i, strlen(reply), reply);
    usb_send_str(s);
    //         m.decodeToken(&reply[1]);
    //         m.toString(&reply[1], MRL-1);
    //         usb_write(reply);
  }
  usb_send_str("<done tokens>\n");
}

///////////////////////////////////////////

int UMissionThread::toTokenString(char * bf, int bfCnt)
{
  return snprintf(bf, bfCnt, "%c%d\n", UMissionLine::MP_THREAD, threadNr);
}

///////////////////////////////////////////

bool UMissionThread::decodeToken(char * line, uint16_t * tn)
{ // decode tiken (from EE-flash)
//   // debug
//    const int MSL=120;
//    char s[MSL];
//   // debug end
  bool isOK = true;
  if (line[0] == UMissionLine::MP_THREAD)
  { // get thread number
    // tokens thread number is on a separate line
    *tn = strtol(&line[1], NULL, 10);
    isOK = lineCnt == 0;
    if (isOK)
      threadNr = *tn;
//     // debug
//      snprintf(s, MSL, "# got new thread %d isOK %d\r\n", threadNr, isOK);
//      usb_send_str(s);
//     // debug end
  }
  else
  { // not a thread number
    int idx = lineStartIndex + lineCnt;
    //     // debug
//      snprintf(s, MSL, "# pre  decode thread %d line %d idx=%d isOK %d, %s\r\n", threadNr, lineCnt, idx, isOK, line);
//      usb_send_str(s);
    //     // debug end
    isOK = miLines[idx].decodeToken(line);
//     // debug
//      snprintf(s, MSL, "# post decode thread %d line %d idx=%d isOK %d\r\n", threadNr, lineCnt, idx, isOK);
//      usb_send_str(s);
//     // debug end
    if (isOK)
    {
      lineCnt++;
      if (idx >= miLinesCnt)
        miLinesCnt++;
    }
  }
  return isOK;
}

////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////


void UMissionLine::clear()
{
  accUse = false;
  logUse = false;
  trUse = false;
  velUse = false;
  edgeLUse = false;
  edgeRUse = false;
  edgeWhiteUse = false;
  balUse = false;
  gotoUse = false;
  drivePosUse = false;
  irSensorUse = false;
  irDistRefUse = false;
  label = 0;
  eventSet = 0;
  //
  distUse = false;
  timeUse = false;
  turnUse = false;
  countUse = false;
  xingWhiteUse = false;
  xingBlackUse = false;
  lineValidUse = false;
  irDist1Use = '\0';
  irDist2Use = '\0';
  tiltUse = false;
  eventMask = 0;
  logFullUse = false;
  // 
  valid = false;
  visits = 0;
}

///////////////////////////////////////////////////

bool UMissionLine::finished(UMissionThread * state, uint16_t * labelNum, bool * endedAtEndAngle)
{
  bool finished;
  bool condition = turnUse or distUse or timeUse or
      xingBlackUse or xingWhiteUse or lineValidUse or 
      irDist1Use or irDist2Use or eventMask or tiltUse or logFullUse;
  // test for finished with this line
  //usb_send_str("# Line::testFinished -> start\n");
  if (condition or gotoUse)
  { // there is a continue condition
    //usb_send_str("# Line::testFinished -> condition or goto\n");
    finished = (userMission.eventFlags & eventMask) > 0;
//     if (finished)
//       usb_send_str("# Line::testFinished -> 2 finished\n");
//     else
//       usb_send_str("# Line::testFinished -> 2 not finished\n");
    if (not finished and turnUse)
    { // avoid angle folding
      float ta = pose[2] - state->turnSumLast;
      if (ta > M_PI)
        ta -= 2* M_PI;
      else if (ta < -M_PI)
        ta += 2 * M_PI; 
      state->turnAngSum += ta;
      state->turnSumLast = pose[2];
      //finished = misAngSum * 180/M_PI > fabs(misLine->turn);
    }
    if (not finished and distUse)
    { 
      if (distUse == '<')
        finished = fabsf(distance - state->linStartDist) < dist;
      else
        finished = fabsf(distance - state->linStartDist) >= dist;
    }
    if (not finished and timeUse)
    {
      finished = time < float(hbTimerCnt - state->misStartTime)/1000.0;
    }
    if (turnUse)
    {
      if (not finished)
      {
        finished = fabsf(state->turnAngSum) * 180/M_PI > fabsf(turn);
        if (finished)
        {
          *endedAtEndAngle = true;
        }
      }
      //             else
      //               mission_turn_ref = pose[2];
    }
    if (not finished and xingBlackUse)
    {
      if (xingBlackUse == '<')
        finished = crossingBlackCnt < xingBlackVal;
      else if (xingBlackUse == '>')
        finished = crossingBlackCnt > xingBlackVal;
      else 
        finished = crossingBlackCnt == xingBlackVal;
      //if (finished)
//         usb_send_str("# xing black\n");
    }
    if (not finished and xingWhiteUse)
    {
      if (xingWhiteUse == '<')
        finished = crossingWhiteCnt < xingWhiteVal;
      if (xingWhiteUse == '>')
        finished = crossingWhiteCnt > xingWhiteVal;
      else
        finished = crossingWhiteCnt == xingWhiteVal;
      // debug
      //             if (crossingWhiteCnt > 0)
      //             {
      //               snprintf(s, MSL, "# xing white %d %d\n", crossingWhiteCnt, finished);
      //               usb_send_str(s);
      //             }
      // debug end
    }
    if (not finished and lineValidUse)
    {
      if (mission_line_LeftEdge)
        finished = lsLeftValid == lineValidVal;
      else
        finished = lsRightValid == lineValidVal;
    }
    if (not finished and irDist1Use)
    {
      if (irDist1Use == '<')
        finished = irDistance[0] < irDist1;
      else
        finished = irDistance[0] >= irDist1;
    }
    if (not finished and irDist2Use)
    {
      if (irDist2Use == '<')
        finished = irDistance[1] < irDist2;
      else
        finished = irDistance[1] >= irDist2;
    }
    // with goto lines as the exception
    if (gotoUse)
    { // this is a goto-line
      if (true)
      { // count is incremented already
        if ((visits <= count or not countUse) and not finished)
        { // we need to jump, return the label number
          *labelNum = gotoDest;
        }
        else
        { // reset count
          visits = 0;
        }
      }
      // debug
//       const int MSL = 90;
//       char s[MSL];
//       snprintf(s, MSL, "#Line::finished visits=%d, count=%d (%d), *label=%d\n",
//                visits, count, countUse, *labelNum);
//       usb_send_str(s);
      // debug end
      finished = true;
    }
    if (not finished and logFullUse)
    {
      finished = logFull;
    }
    if (not finished and tiltUse)
    {
      if (tiltUse == '<')
        finished = pose[3] < tilt;
      else
        finished = pose[3] >= tilt;
    }
//     if (finished)
//       usb_write("# mission line - finished\n");
//     else
//       usb_write("# mission line - not finished\n");
  }
  else
  { // no condition (or just count), so continue right away
    finished = true;
    //usb_write("# mission line - no condition\n");
  }
  return finished;
}

///////////////////////////////////////////////////////

void UMissionLine::postProcess(float lineStartAngle, bool endAtAngle)
{
  // turn off one-liner controls
  // edge follow, wall follow and turn
  if (turnUse or edgeLUse or edgeRUse or trUse or irSensorUse == 1)
  {
    if (turnUse and endAtAngle)
    { // ended at an angle condition, so we know
      // that new angle ref should be exactly this angle
      mission_turn_ref = lineStartAngle + turn * M_PI / 180.0;
      while (mission_turn_ref > M_PI)
        mission_turn_ref -= 2 * M_PI;
      while (mission_turn_ref < -M_PI)
        mission_turn_ref += 2 * M_PI;
    }
    else 
    { // not a specific turn angle, but turning allowed, so use 
      // current heading as new reference
      mission_turn_ref = pose[2];
    }
    if (edgeLUse or edgeRUse)
      regul_line_use = false;
    if (trUse)
      mission_turn_do = false;
    if (irSensorUse) 
    { // ir-sensor drive control ended
      mission_wall_use = false;
    }  
  }
  // use of position controller is a one-liner also
  if (drivePosUse)
    mission_pos_use = false;
}



///////////////////////////////////////////////////////

void UMissionLine::implementLine()
{ // implement all line parameters
  // and note start state
  if (logUse and not logFull)
  { // logging may have changed
    if (log > 0)
    { // (re)start logging
      startLogging(log, false);
    }
    else
    { // pause logging, when set to log=0
      stopLogging();
    }
  }
  if (balUse)
    balance_active = bal;
  //
  if (irSensorUse)
  { // there is new 
    if (irSensor == 1)
    { // sensor 1 is for wall follow
      mission_wall_turn = true; // turn is sensor 1 (side looking)
    }
    else
    { // we are dooing - follow the leader - i.e. speed control
      mission_wall_turn = false; // is sensor 2 (fwd looking)
    }
    if (irDistRefUse)
      mission_wall_ref = irDistRef;
    mission_wall_use = true;
  }
//   else
//     mission_wall_use = false;
  //
  if (velUse)
  {  // change velocity
    mission_vel_ref = vel;
    //usb_send_str("# set mission_vel_ref\n");
  }
  if (drivePosUse)
  {
    mission_pos_ref = drivePos;
    mission_pos_use = true;
  }
  // edge/line sensor
  //lineSensorOn = false;
  if (xingWhiteUse or xingBlackUse)
    // turn on sensor for crossing detect
    lineSensorOn = true;
  if (edgeLUse or edgeRUse)
  {
    lineSensorOn = true;
    regul_line_use = true;
    mission_line_LeftEdge = edgeLUse;
    mission_line_ref = edgeRef;
    if (edgeWhiteUse)
      lsIsWhite = edgeWhite;
  }
//   else if (regul_line_use)
//   { // set heading ref to as is now
//     regul_line_use = false;
//   }
  // turn control
  if (trUse)
  { // turn radius in meters positive i left
    mission_turn_radius = tr;
    mission_turn_do = true;
    // set turn angle
    mission_tr_turn = turn;
    // reduce reference velocity
    // mission_vel_ref_preturn = mission_vel_ref;
    // reduce reference (mid-point) velocity and allow outher wheel to keep old velocity
    // especially when turn radius is very small -> 0
    // ¤¤¤              mission_vel_ref = mission_vel_ref_preturn * mission_turn_radius / 
    //                 (mission_turn_radius + odoWheelBase/2.0);
  }
//   else
//   { // no turn this time - keep current turn ref
//     mission_turn_do = false;
//     //               regul_turn_vel_reduc[0] = 0;
//     //               regul_turn_vel_reduc[1] = 0;
//   }
  if (accUse)
    regul_acc_limit = acc;
  if (eventSet > 0)
  { // there is at least one user event, activate
    for (int i = 0; i < 32; i++)
    { // test all 32 possible user events
      if ((eventSet & (1 << i)) > 0)
        userMission.setEvent(i);
    }
  }
}


///////////////////////////////////////////////////////

int UMissionLine::toString(char* bf, int bfCnt, bool frame)
{
  char * ms = bf;
  char * mc;
  int n = 0;
  const char sep = ',';
  if (not valid)
  {
    strncpy(ms, "\r# not a valid line", bfCnt - n);
    n += strlen(ms);
    ms = &bf[n];
  }
  else if (frame)
  {
    strcpy(ms, "<m ");
    n+=3;
    ms = &bf[n];
  }
  if (velUse)
  {
    if (n > 3) {*ms++=sep; n++;}
    snprintf(ms, bfCnt - n, "vel=%g", vel);
    n += strlen(ms);
    ms = &bf[n];
  }
  if (accUse)
  {
    if (n > 3) {*ms++=sep; n++;}
    snprintf(ms, bfCnt - n, "acc=%g", acc);
    n += strlen(ms);
    ms = &bf[n];
  }
  if (trUse)
  {
    if (n > 3) {*ms++=sep; n++;}
    snprintf(ms, bfCnt - n - 1, "tr=%g", tr);
    n += strlen(ms);
    ms = &bf[n];
  }
  if (edgeLUse)
  {
    if (n > 3) {*ms++=sep; n++;}
    snprintf(ms, bfCnt - n - 1, "edgeL=%g", edgeRef);
    n += strlen(ms);
    ms = &bf[n];
  }
  if (edgeRUse)
  {
    if (n > 3) {*ms++=sep; n++;}
    snprintf(ms, bfCnt - n - 1, "edgeR=%g", edgeRef);
    n += strlen(ms);
    ms = &bf[n];
  }
  if (edgeWhiteUse)
  {
    if (n > 3) {*ms++=sep; n++;}
    snprintf(ms, bfCnt - n - 1, "white=%d", edgeWhite);
    n += strlen(ms);
    ms = &bf[n];
  }
  if (logUse)
  {
    if (n > 3) {*ms++=sep; n++;}
    snprintf(ms, bfCnt - n - 1, "log=%g", log);
    n += strlen(ms);
    ms = &bf[n];
  }
  if (balUse)
  {
    if (n > 3) {*ms++=sep; n++;}
    snprintf(ms, bfCnt - n - 1, "bal=%d", bal);
    n += strlen(ms);
    ms = &bf[n];
  }
  if (irSensorUse)
  {
    if (n > 3) {*ms++=sep; n++;}
    snprintf(ms, bfCnt - n - 1, "irsensor=%d", irSensor);
    n += strlen(ms);
    ms = &bf[n];
  }
  if (irDistRefUse)
  {
    if (n > 3) {*ms++=sep; n++;}
    snprintf(ms, bfCnt - n - 1, "irdist=%g", irDistRef);
    n += strlen(ms);
    ms = &bf[n];
  }
  if (drivePosUse)
  {
    if (n > 3) {*ms++=sep; n++;}
    snprintf(ms, bfCnt - n - 1, "topos=%g", drivePos);
    n += strlen(ms);
    ms = &bf[n];
  }
  if (label > 0)
  {
    if (n > 3) {*ms++=sep; n++;}
    snprintf(ms, bfCnt - n - 1, "label=%d", label);
    n += strlen(ms);
    ms = &bf[n];
  }
  if (gotoUse)
  {
    if (n > 3) {*ms++=sep; n++;}
    snprintf(ms, bfCnt - n - 1, "goto=%d", gotoDest);
    n += strlen(ms);
    ms = &bf[n];
  }
  if (eventSet)
  { // there  is at least one event set on this line
    for (int i = 0; i < 32; i++)
    { // test all possible events
      if (eventSet & (1 << i))
      { // event i is set, add to line
        if (n > 3) {*ms++=sep; n++;}
        snprintf(ms, bfCnt - n - 1, "event=%d", i);
        n += strlen(ms);
        ms = &bf[n];
      }
    }
  }
  // now the condition part - if it exist
  mc = ms;
  *mc++ = ':';
  n++;
  if (distUse)
  {
    if (mc - ms > 1) {*mc++=sep; n++;}
    snprintf(mc, bfCnt - n - 1, "dist%c%g", distUse, dist);
    n += strlen(mc);
    mc = &bf[n];
  }
  if (timeUse)
  {
    if (mc - ms > 1) {*mc++=sep; n++;}
    snprintf(mc, bfCnt - n - 1, "time%c%g", timeUse, time);
    n += strlen(mc);
    mc = &bf[n];
  }
  if (turnUse)
  {
    if (mc - ms > 1) {*mc++=sep; n++;}
    snprintf(mc, bfCnt - n - 1, "turn%c%g", turnUse, turn);
    n += strlen(mc);
    mc = &bf[n];
  }
  if (countUse)
  {
    if (mc - ms > 1) {*mc++=sep; n++;}
    snprintf(mc, bfCnt - n - 1, "count%c%d", countUse, count);
    n += strlen(mc);
    mc = &bf[n];
  }
  if (xingBlackUse)
  {
    if (mc - ms > 1) {*mc++=sep; n++;}
    snprintf(mc, bfCnt - n - 1, "xb%c%d", xingBlackUse, xingBlackVal);
    n += strlen(mc);
    mc = &bf[n];
  }
  if (xingWhiteUse)
  {
    if (mc - ms > 1) {*mc++=sep; n++;}
    snprintf(mc, bfCnt - n - 1, "xw%c%d", xingWhiteUse, xingWhiteVal);
    n += strlen(mc);
    mc = &bf[n];
  }
  if (lineValidUse)
  {
    if (mc - ms > 1) {*mc++=sep; n++;}
    snprintf(mc, bfCnt - n - 1, "lv%c%d", lineValidUse, lineValidVal);
    n += strlen(mc);
    mc = &bf[n];
  }
  if (irDist1Use)
  {
    if (mc - ms > 1) {*mc++=sep; n++;}
    snprintf(mc, bfCnt - n - 1, "ir1%c%g", irDist1Use, irDist1);
    n += strlen(mc);
    mc = &bf[n];
  }
  if (irDist2Use)
  {
    if (mc - ms > 1) {*mc++=sep; n++;}
    snprintf(mc, bfCnt - n - 1, "ir2%c%g", irDist2Use, irDist2);
    n += strlen(mc);
    mc = &bf[n];
  }
  if (tiltUse)
  {
    if (mc - ms > 1) {*mc++=sep; n++;}
    snprintf(mc, bfCnt - n - 1, "tilt%c%g", tiltUse, tilt);
    n += strlen(mc);
    mc = &bf[n];
  }
  if (eventMask)
  {
    for (int i = 0; i < 32; i++)
    {
      if (eventMask & (1 << i))
      {
        if (n > 3) {*mc++=sep; n++;}
        snprintf(mc, bfCnt - n - 1, "event=%d", i);
        n += strlen(mc);
        mc = &bf[n];
      }
    }
  }
  if (logFullUse)
  {
    if (mc - ms > 1) {*mc++=sep; n++;}
    snprintf(mc, bfCnt - n - 1, "log=0");
    n += strlen(mc);
    mc = &bf[n];
  }
  if (frame)
  {
    strncpy(mc, "\n\r", bfCnt - n - 1);
    n += strlen(mc);
  }
  return n;
}

/////////////////////////////////////////////////

int UMissionLine::toTokenString(char* bf, int bfCnt)
{
  char * ms = bf;
  char * mc;
  int n = 0;
//   const char sep = ',';
  if (not valid)
  {
    ms[0] = '\0';
  }
  else
  {
    if (velUse)
    {
      snprintf(ms, bfCnt - n, "%c%g", MP_VEL, vel);
      n += strlen(ms);
      ms = &bf[n];
    }
    if (accUse)
    {
      snprintf(ms, bfCnt - n, "%c%g", MP_ACC, acc);
      n += strlen(ms);
      ms = &bf[n];
    }
    if (trUse)
    {
      snprintf(ms, bfCnt - n - 1, "%c%g", MP_TR, tr);
      n += strlen(ms);
      ms = &bf[n];
    }
    if (edgeWhiteUse)
    {
      snprintf(ms, bfCnt - n - 1, "%c%d", MP_EDGE_WHITE, edgeWhite);
      n += strlen(ms);
      ms = &bf[n];
    }
    if (edgeLUse)
    {
      snprintf(ms, bfCnt - n - 1, "%c%g", MP_EDGE_L, edgeRef);
      n += strlen(ms);
      ms = &bf[n];
    }
    if (edgeRUse)
    {
      snprintf(ms, bfCnt - n - 1, "%c%g", MP_EDGE_R, edgeRef);
      n += strlen(ms);
      ms = &bf[n];
    }
    if (logUse)
    {
      snprintf(ms, bfCnt - n - 1, "%c%g", MP_LOG, log);
      n += strlen(ms);
      ms = &bf[n];
    }
    if (balUse)
    {
      snprintf(ms, bfCnt - n - 1, "%c%d", MP_BAL, bal);
      n += strlen(ms);
      ms = &bf[n];
    }
    if (irSensorUse)
    {
      snprintf(ms, bfCnt - n - 1, "%c%d", MP_IR_SENSOR, irSensor);
      n += strlen(ms);
      ms = &bf[n];
    }
    if (irDistRefUse)
    {
      snprintf(ms, bfCnt - n - 1, "%c%g", MP_IR_DIST, irDistRef);
      n += strlen(ms);
      ms = &bf[n];
    }
    if (drivePosUse)
    {
      snprintf(ms, bfCnt - n - 1, "%c%g", MP_DRIVE_DIST, drivePos);
      n += strlen(ms);
      ms = &bf[n];
    }
    if (label > 0)
    {
      snprintf(ms, bfCnt - n - 1, "%c%d", MP_LABEL, label);
      n += strlen(ms);
      ms = &bf[n];
    }
    if (gotoUse)
    {
      snprintf(ms, bfCnt - n - 1, "%c%d", MP_GOTO, gotoDest);
      n += strlen(ms);
      ms = &bf[n]; 
//       usb_send_str("#gotoline\n");
//       usb_send_str(bf);
//       usb_send_str("\n");
    }
    if (eventSet)
    {
      snprintf(ms, bfCnt - n - 1, "%c%lu", MP_EVENT, eventSet);
      n += strlen(ms);
      ms = &bf[n];
    }      
    // not the condition part - if it exist
    mc = ms;
    *mc++ = ':';
    n++;
    if (distUse)
    {
      snprintf(mc, bfCnt - n - 1, "%c%c%g", MC_DIST, distUse, dist);
      n += strlen(mc);
      mc = &bf[n];
    }
    if (timeUse)
    {
      snprintf(mc, bfCnt - n - 1, "%c%c%g", MC_TIME, timeUse, time);
      n += strlen(mc);
      mc = &bf[n];
    }
    if (turnUse)
    {
      snprintf(mc, bfCnt - n - 1, "%c%c%g", MC_TURN, turnUse, turn);
      n += strlen(mc);
      mc = &bf[n];
    }
    if (countUse)
    {
      snprintf(mc, bfCnt - n - 1, "%c%c%d", MC_COUNT, countUse, count);
      n += strlen(mc);
      mc = &bf[n];
    }
    if (xingBlackUse)
    {
      snprintf(mc, bfCnt - n - 1, "%c%c%d", MC_XINGB, xingBlackUse, xingBlackVal);
      n += strlen(mc);
      mc = &bf[n];
    }
    if (xingWhiteUse)
    {
      snprintf(mc, bfCnt - n - 1, "%c%c%d", MC_XINGW, xingWhiteUse, xingWhiteVal);
      n += strlen(mc);
      mc = &bf[n];
    }
    if (lineValidUse)
    {
      snprintf(mc, bfCnt - n - 1, "%c%c%d", MC_LINE_VALID, lineValidUse, lineValidVal);
      n += strlen(mc);
      mc = &bf[n];
    }
    if (irDist1Use)
    {
      snprintf(mc, bfCnt - n - 1, "%c%c%g", MC_IR_DIST1, irDist1Use, irDist1);
      n += strlen(mc);
      mc = &bf[n];
    }
    if (irDist2Use)
    {
      snprintf(mc, bfCnt - n - 1, "%c%c%g", MC_IR_DIST2, irDist2Use, irDist2);
      n += strlen(mc);
      mc = &bf[n];
    }
    if (tiltUse)
    {
      snprintf(mc, bfCnt - n - 1, "%c%c%g", MC_TILT, tiltUse, tilt);
      n += strlen(mc);
      mc = &bf[n];
    }
    if (logFullUse)
    {
      snprintf(mc, bfCnt - n - 1, "%c0", MC_LOG);
      n += strlen(mc);
      mc = &bf[n];
    }
    if (eventMask)
    {
      snprintf(mc, bfCnt - n - 1, "%c%lu", MC_EVENT, eventMask);
      n += strlen(mc);
      mc = &bf[n];
    }
    strncpy(mc, "\n", bfCnt - n - 1);
    n += strlen(mc);
  }
  return n;
}




////////////////////////////////////////////////////////////////

bool UMissionLine::decodeLine(const char* buffer, int16_t * threadNumber)
{
  char * p1 = (char *)buffer;
  char * p2 = strchr(p1, ':');
  char * p3 = strchr(p1, '=');
  bool err = false;
  // reset use flags
  clear();
  missionErrStr[0] = '\0';
  // strip white space
  while (*p1 <= ' ' and *p1 > '\0') p1++;
  // find all parameters until ':'
  while ((p1 < p2 or p2 == NULL) and p3 != NULL and not err)
  {
    // debug
//     const int MSL = 100;
//     char s[MSL];
//     snprintf(s, MSL, "#Decoding line p1='%s'\n # p3 = '%s'\n", p1, p3);
//     usb_send_str(s);
    // debug end
    if (strncmp (p1, "acc", 3) == 0)
    {
      accUse = true;
      acc = strtof(++p3, &p1);
    }
    else if (strncmp (p1, "vel", 3) == 0)
    {
      velUse = true;
      vel = strtof(++p3, &p1);
    }
    else if (strncmp(p1, "tr", 2) == 0)
    {
      trUse = true;
      tr = fabsf(strtof(++p3, &p1));
    }
    else if (strncmp(p1, "edgel", 5) == 0 or strncmp(p1, "liner", 5) == 0)
    {
      edgeLUse = true;
      edgeRef = strtof(++p3, &p1);
      if (edgeRef > 2.0)
        edgeRef = 2.0;
      if (edgeRef < -2.0)
        edgeRef = -2.0;
    }
    else if (strncmp(p1, "edger", 5) == 0 or strncmp(p1, "liner", 5) == 0)
    {
      edgeRUse = true;
      edgeRef = strtof(++p3, &p1);
      if (edgeRef > 2.0)
        edgeRef = 2.0;
      if (edgeRef < -2.0)
        edgeRef = -2.0;
    }
    else if (strncmp(p1, "white", 5) == 0)
    {
      edgeWhiteUse = true;
      edgeWhite = strtol(++p3, &p1, 10);
      if (edgeWhite)
        edgeWhite = true;
    }
    else if (strncmp (p1, "log", 3) == 0)
    {
      logUse = true;
      log = strtof(++p3, &p1);
    }
    else if (strncmp (p1, "bal", 3) == 0)
    {
      balUse = true;
      bal = strtof(++p3, &p1) > 0.5;
    }
    else if (strncmp (p1, "irsensor", 8) == 0)
    {
      irSensorUse = true;
      irSensor = strtol(++p3, &p1, 10);
      if (irSensor > 2)
        irSensor = 2;
      else if (irSensor < 1)
        irSensor = 1;
    }
    else if (strncmp (p1, "irdist", 6) == 0)
    {
      irDistRefUse = true;
      irDistRef = strtof(++p3, &p1);
    }
    else if (strncmp (p1, "topos", 5) == 0)
    {
      drivePosUse = true;
      drivePos = strtof(++p3, &p1);
    }
    else if (strncmp (p1, "label", 5) == 0)
    {
      label = strtol(++p3, &p1, 10);
    }
    else if (strncmp (p1, "goto", 4) == 0)
    {
      // debug
//       const int MSL = 100;
//       char s[MSL];
//       snprintf(s, MSL, "#Decoding goto p1='%s', p3 = '%s'\n", p1, p3);
//       usb_send_str(s);
      // debug end
      gotoUse = true;
      gotoDest = strtol(++p3, &p1, 0);
//       snprintf(s, MSL, "#Decoding goto use=%d, dest = %d\n", gotoUse, gotoDest);
//       usb_send_str(s);
    }
    else if (strncmp (p1, "thread", 6) == 0)
    {
      *threadNumber = strtol(++p3, &p1, 0);
    }
    else if (strncmp (p1, "event", 5) == 0)
    {
      int s = strtol(++p3, &p1, 0);
      if (s >= 0 and s < 32)
        eventSet |= 1 << s;
    }
    else
    { // error, just skip
      snprintf(missionErrStr, missionErrStrMaxCnt, "failed parameter at %s", p1);
      p1 = ++p3;
      err = true;
    }
    // remove white space
    while ((*p1 <= ' ' or *p1 == ',') and *p1 > '\0') p1++;
    p3 = strchr(p1, '=');
    
  }
  if (p2 != NULL)
  {
    p1 = p2 + 1;
    while (*p1 <= ' ' and *p1 > '\0') p1++;
    p3 = strchr(p1, '=');
    if (p3 == NULL)
      p3 = strchr(p1, '<');
    if (p3 == NULL)
      p3 = strchr(p1, '>');    
    while (*p1 > '\0' and p3 != NULL and not err)
    {
      // debug
//       const int MSL = 100;
//       char s[MSL];
//       snprintf(s, MSL, "#Decoding condition p1='%s', p3 = '%s'\n", p1, p3);
//       usb_send_str(s);
      // debug end
      if (strncmp (p1, "dist", 4) == 0)
      { // distance is always positive (even if reversing)
        distUse = *p3;
        dist = fabsf(strtof(++p3, &p1));
      }
      else if (strncmp (p1, "turn", 4) == 0)
      {
        turnUse = *p3;
        turn = strtof(++p3, &p1);
      }
      else if (strncmp (p1, "time", 4) == 0)
      {
        timeUse = *p3;
        time = strtof(++p3, &p1);
      }
      else if (strncmp (p1, "count", 4) == 0)
      {
        countUse = *p3;
        count = strtol(++p3, &p1, 10);
      }
      else if (strncmp (p1, "xb", 2) == 0)
      {
        xingBlackUse = *p3;
        xingBlackVal = strtol(++p3, &p1, 10);
      }
      else if (strncmp (p1, "xw", 2) == 0)
      {
        xingWhiteUse = *p3;
        xingWhiteVal = strtol(++p3, &p1, 10);
      }
      else if (strncmp (p1, "lv", 2) == 0)
      {
        lineValidUse = *p3;
        lineValidVal = strtol(++p3, &p1, 10);
      }
      else if (strncmp (p1, "ir1", 3) == 0)
      {
        irDist1Use = *p3;
        irDist1 = strtof(++p3, &p1);
        //usb_send_str("#ir1\n");
      }
      else if (strncmp (p1, "ir2", 3) == 0)
      {
        irDist2Use = *p3;
        irDist2 = strtof(++p3, &p1);
        //usb_send_str("#ir2\n");
      }
      else if (strncmp (p1, "tilt", 4) == 0)
      {
        tiltUse = *p3;
        tilt = strtof(++p3, &p1);
      }
      else if (strncmp (p1, "log", 3) == 0)
      {
        logFullUse = true;
        p1 = p3 + 2; // skip zero
      }
      else if (strncmp (p1, "event", 5) == 0)
      {
        int s = strtol(++p3, &p1, 0);
        if (s < 32 and s >= 0)
          eventMask |= 1 << s;
      }
      else
      { // error, just skip
        snprintf(missionErrStr, missionErrStrMaxCnt, "failed condition at %s", p1);
        p1 = ++p3;
        err = true;
      }
      // remove white space
      while ((*p1 <= ' ' or *p1 == ',') and *p1 > '\0') p1++;
      p3 = strchr(p1, '=');
      if (p3 == NULL)
        p3 = strchr(p1, '<');
      if (p3 == NULL)
        p3 = strchr(p1, '>');    
    }
  }
  valid = not err;
  
//   if (true)
//   {
//     const int MSL = 180;
//     char s[MSL];
//     snprintf(s, MSL, "# decoded ir1Use=%c, ir2use=%c from %s\n", irDist1Use, irDist2Use, buffer);
//     usb_send_str(s);
//   }
  
  return valid;
}

bool UMissionLine::decodeToken(const char* buffer)
{
  char * p1 = (char *)buffer;
  char * p2 = strchr(p1, ':');
  bool err = false;
  uint8_t n = 255;
  // reset use flags
  clear();
//     const int MSL = 120;
//     char  s[MSL];
//   missionErrStr[0]='\0';
  // find all parameters until ':'
  while (p1 < p2)
  {
    // debug
//      snprintf(s, MSL, "#pi1=%s ¤ p2=%s\r\n", p1, p2);
//      usb_send_str(s);
    // denug end
    if (*p1 == MP_ACC) 
    {
      accUse = true;
      acc = strtof(++p1, &p1);
    }
    else if (*p1 == MP_VEL)
    {
      velUse = true;
      vel = strtof(++p1, &p1);
    }
    else if (*p1 == MP_TR)
    {
      trUse = true;
      tr = fabsf(strtof(++p1, &p1));
    }
    else if (*p1 == MP_EDGE_L)
    {
      edgeLUse = true;
      edgeRef = strtof(++p1, &p1);
    }
    else if (*p1 == MP_EDGE_R)
    {
      edgeRUse = true;
      edgeRef = strtof(++p1, &p1);
    }
    else if (*p1 == MP_EDGE_WHITE)
    {
      edgeWhiteUse = true;
      edgeWhite = strtol(++p1, &p1, 10);
    }
    else if (*p1 == MP_LOG)
    {
      logUse = true;
      log = strtof(++p1, &p1);
    }
    else if (*p1 == MP_BAL)
    {
      balUse = true;
      bal = strtol(++p1, &p1, 10);
    }
    else if (*p1 == MP_IR_SENSOR)
    {
      irSensorUse = true;
      irSensor = strtol(++p1, &p1, 10);
    }
    else if (*p1 == MP_IR_DIST)
    {
      irDistRefUse = true;
      irDistRef = strtof(++p1, &p1);
    }
    else if (*p1 == MP_DRIVE_DIST)
    {
      drivePosUse = true;
      drivePos = strtof(++p1, &p1);
    }
    else if (*p1 == MP_LABEL)
    {
      label = strtol(++p1, &p1, 10);
    }
    else if (*p1 == MP_GOTO)
    {
      gotoUse = true;
      gotoDest = strtol(++p1, &p1, 10);
    }
    else if (*p1 == MP_EVENT)
    {
      eventSet = strtol(++p1, &p1, 10);
    }
    else if (*p1 > ' ')
    { // error, just skip
      err = true;
      snprintf(missionErrStr, missionErrStrMaxCnt, "failed line P at %s\n", p1);
      break;
    }
    // remove seperator
//     if (*p1 == ',')
//       p1++;
  }
  if (p2 != NULL and not err)
  {
    p1 = p2 + 1;
    while (*p1 > ' ')
    {
      // debug
//       snprintf(s, MSL, "#pi1=%s\n", p1);
//       usb_send_str(s);
      // debug end
      if (*p1 == MC_DIST)
      { // distance is always positive (even if reversing)
        distUse = *(++p1);
        dist = fabsf(strtof(++p1, &p1));
      }
      else if (*p1 == MC_TURN)
      {
        turnUse = *(++p1);
        turn = strtof(++p1, &p1);
      }
      else if (*p1 == MC_TIME)
      {
        timeUse = *(++p1);
        time = strtof(++p1, &p1);
      }
      else if (*p1 == MC_COUNT)
      {
        countUse = *(++p1);;
        count = strtol(++p1, &p1, 10);
      }
      else if (*p1 == MC_XINGB)
      {
        xingBlackUse = *(++p1);
        xingBlackVal = strtol(++p1, &p1, 10);
      }
      else if (*p1 == MC_XINGW)
      {
        xingWhiteUse = *(++p1);
        xingWhiteVal = strtol(++p1, &p1, 10);
      }
      else if (*p1 == MC_LINE_VALID)
      {
        lineValidUse = *(++p1);
        lineValidVal = strtol(++p1, &p1, 10);
      }
      else if (*p1 == MC_IR_DIST1)
      {
        irDist1Use = *(++p1);
        irDist1 = strtof(++p1, &p1);
      }
      else if (*p1 == MC_IR_DIST2)
      {
        irDist2Use = *(++p1);
        irDist2 = strtof(++p1, &p1);
      }
      else if (*p1 == MC_TILT)
      {
        tiltUse = *(++p1);
        tilt = strtof(++p1, &p1);
      }
      else if (*p1 == MC_LOG)
      {
        logFullUse = true;
        p1++;
        p1++;
        while (isdigit(*p1))
          p1++;
      }
      else if (*p1 == MC_EVENT)
      {
        eventMask = strtol(++p1, &p1, 10);
      }
      else
      { // error, just skip
        err = true;
        snprintf(missionErrStr, missionErrStrMaxCnt, "failed line C at %s (n=%d)\n", p1, n);
        break;
      }
//       if (*p1 > ' ' and *p1 == ',')
//         p1++;
    }
  }
  valid = not err;
  return valid;
}


/////////////////////////////////////////////////
/////////////////////////////////////////////////
/////////////////////////////////////////////////
/////////////////////////////////////////////////

bool UMission::testFinished()
{
  bool theEnd = true;
  bool lineFinished[threadsMax];
//  bool debugOut = false;
  //usb_send_str("# testFinished -> test start\n");
  for (int i = 0; i < threadsCnt; i++)
  {
    lineFinished[i] = threads[i].testFinished();
//     if (lineFinished[i])
//       usb_send_str("# testFinished -> finished thread\n");
//    debugOut |= lineFinished[i];
  }
//   if (debugOut)
//   {
//     const int MSL = 50;
//     char s[MSL];
//     for (int i=0; i < threadsCnt; i++)
//     {
//       snprintf(s, MSL, "#line end th:%d line:%d finished:%d\n",
//                i, threads[i].misLineNum, lineFinished[i]);
//       usb_send_str(s);
//     }
//   }
  // reset events before implementing next line
  eventFlags = 0;
//  usb_send_str("# testFinished -> events cleared\n");
  // implement next line (if relevant)
  for (int i = 0; i < threadsCnt; i++)
  {
    if (lineFinished[i])
    {
      theEnd &= threads[i].moveToNextLine();
//       if (theEnd)
//         usb_send_str("# testFinished -> finished all threads\n");
    }
    else
      theEnd = false;
  }
  // event flag 0 has special meening : stop mission NOW!
  if ((eventFlags & 0x01) > 0)
    theEnd = true;
  return theEnd;
}


bool UMission::startMission()
{
  // debug
//   const int MSL = 120;
//   char s[MSL];
  // debug end
  bool isOK = threadsCnt > 0;
  if (isOK)
  {
    time = 0.0;
    clearPose();
    balance_active = false;
    mission_vel_ref = 0;
    mission_turn_ref = 0;
    // reset log buffer (but do not start logging)
    initLogStructure(50);
    logAllow = true;
    toLog = false;
    lineSensorOn = false;
    mission_wall_turn = false;
    mission_wall_use = false;
    mission_vel_ref = 0.2; // default speed
    lineSensorOn = false;
    regul_line_use = false;
    regul_acc_limit = 3.0;
    motorSetEnable(true, true);
    //
    for (int i = 0; i < threadsCnt; i++)
    {
      threads[i].startMission();
//       threads[i].resetVisits();
//       threads[i].misLineNum = 0;
//       threads[i].implementNewLine();
//       // debug
//       snprintf(s, MSL, "# starting thread %d (%d)\r\n", i, threads[i].threadNr);
//       usb_send_str(s);
//       // debug end
    }
  }
  return isOK;
}

///////////////////////////////////////////////////

// bool UMission::eeAddBlock(char * data, int dataCnt)
// {
//   if (eeConfig.getAddr() + dataCnt < 2048 - 2)
//   {
//     eeConfig.busy_wait();
//     eeConfig.write_block(data, dataCnt);
//     return true;
//   }
//   else
//     return false;
// }

///////////////////////////////////////////////////////

void UMission::eePromSaveMission()
{
  const int MRL = 100;
  char reply[MRL];
//   const int MSL = 100;
//   char s[MSL];
  int n, i = 0, t = 0;
  uint32_t adr = eeConfig.getAddr();
  bool isOK = true;
  // reserve space for size
  eeConfig.setAddr(adr + 2);
  //eePushAdr += 2;
  for (t = 0; t < threadsCnt; t++)
  {
    UMissionThread * mt = &threads[t];
    n = mt->toTokenString(reply, MRL);
    isOK = eeConfig.pushBlock(reply, n);
//     // debug
//      snprintf(reply, MRL, "# saving thread %d (%d), %d line(s) ...\r\n", t, mt->threadNr, mt->lineCnt);
//      usb_send_str(reply);
//     // debug end
    for (i = 0; i < mt->lineCnt and isOK; i++)
    {
      n = miLines[mt->lineStartIndex + i].toTokenString(reply, MRL);
      isOK = eeConfig.pushBlock(reply, n);
//       // debug
//        snprintf(s, MSL, "# line %d OK=%d (%d bytes) %s\r\n", i, isOK, n, reply);
//        usb_send_str(s);
//       // debug end
    }
//     // debug
//     usb_send_str("# end\r\n");
//     // debug end
    if (not isOK) 
      break;
  }
  if (not isOK)
  { // not enough space
    snprintf(reply, MRL, "# failed to save mission thread %d line %d (of %d)\n", t, i, miLinesCnt);
    usb_send_str(reply);
  }
  // write size of (saved part of) misson in characters
  eeConfig.write_word(adr, eeConfig.getAddr() - adr);
}

////////////////////////////////////

void UMission::eePromLoadMission()
{
  const int MRL = 100;
  char reply[MRL];
//   const int MSL = 120;
//   char s[MSL];
//   const int MSL = 100;
//   char s[MSL];
  int n = 0, m;
  uint16_t t = -1;
  miLinesCnt = 0;
  bool isOK = true;
  UMissionThread * mt;
  // read number of bytes in mission
  clear();
  threadsCnt = 1;
  mt = threads; // get first thread
  m = eeConfig.readWord() - 1;
  while (m > 1 and n < MRL and isOK)
  {
    snprintf(missionErrStr, 10, "#none\n"); // no error
    // read one byte at a time
    reply[n] = eeConfig.readByte();
    // debug
//     reply[n+1] = '\n';
//     reply[n+2] = '\0';
//     usb_send_str(reply);
    // debug end
    m--;
    if (reply[n] <= ' ')
    { // newline marks end of line
      if (n > 1)
      { // this is a line
        isOK = mt->decodeToken(reply, &t);
        if (not isOK and (t != mt->threadNr))
        { // line belongs to next thread
          mt++;
          threadsCnt++;
          mt->lineStartIndex = miLinesCnt;
          isOK = mt->decodeToken(reply, &t);
          // debug
//          usb_send_str("# new thread\r\n");
          // debug end
        }
      }
//       // debug
//       snprintf(s, MSL, "# m=%d reading to thread %d - %d chars %c%c%c #... - isOK %d\r\n", 
//                m, mt->threadNr, n, reply[0], reply[1], reply[2], isOK);
//       usb_send_str(s);
//       // debug end
      n = 0;
    }
    else
      n++;
  }
  if (not isOK)
    usb_send_str(missionErrStr);
}

/////////////////////////////////////////////////

bool UMission::addLine(const char * lineToAdd)
{
  int16_t tn, ti;
  bool isOK;
  if (threadsCnt == 0)
    threadsCnt = 1;
  UMissionThread * mt = &threads[threadsCnt - 1];
  tn = mt->threadNr;
  isOK = mt->addLine(lineToAdd, &tn);
  if (not isOK and tn != mt->threadNr and threadsCnt < threadsMax - 1)
  { // new thread
    if (tn <= 0)
      // threads should have a number >= 1
      tn = 0;
    else
      tn++;
    ti = getThreadIndex(tn);
    isOK = ti == -1;
    if (isOK)
    { // thread number not seen before
      threadsCnt++;
      // advance to next thread
      mt++;
      // set start line for this thread
      mt->lineStartIndex = miLinesCnt;
      //mt->threadNr = tn;
      // add other parts of thread (if any)
      isOK = mt->addLine(lineToAdd, &tn);
    }
    else
      usb_send_str("# dublicated thread number - skipped thread line\n");
  }
  return isOK;
}

////////////////////////////////////////////////

bool UMission::modLine(int16_t thread, int16_t line, const char * p2)
{
  int idx = getThreadIndex(thread);
  bool isOK = idx >= 0;
  // debug
//   const int MSL = 50;
//   char s[MSL];
//   snprintf(s, MSL, "# found thread %d, isOK=%d (idx=%d)\n", thread, isOK, idx);
//   usb_send_str(s);
  // debug end
  if (isOK)
  {
    UMissionThread * mt = &threads[idx];
    isOK = mt->modLine(line, p2);
  }  
  return isOK;
}



////////////////////////////////////////////////

void UMission::getLines()
{
  for (int i = 0; i < threadsCnt; i++)
  {
    threads[i].getLines();
  }
}

//////////////////////////////////////

void UMission::getToken()
{
  for (int t = 0; t < threadsCnt; t++)
    threads[t].getToken();
}

//////////////////////////////////////
/**
 * Get number of threads */
int UMission::getLinesCnt()
{
  return miLinesCnt;
}

//////////////////////////////////////

void UMission::clear()
{
  miLinesCnt = 0;
  for (int i = 0; i < miLinesCntMax; i++)
    miLines[i].clear();
  threadsCnt = 0;
  for (int i = 0; i < threadsMax; i++)
    threads[i].clear(i + 1);
}

//////////////////////////////////////////

int16_t UMission::getThreadIndex ( int16_t thread )
{
  int16_t idx = -1;
  for (int i = 0; i < threadsCnt; i++)
  {
    if (threads[i].threadNr == thread)
    {
      idx = i;
      break;
    }
  }
  return idx;
}

/////////////////////////////////////////

void UMission::decodeEvent(const char* eventNumber)
{
  const char * p1 = eventNumber;
  int e;
  while (*p1 == ' ' or *p1 == '=')
    p1++;
  e = strtol(p1, NULL, 10);
  if (e >= 0 and e < 32)
    setEvent(e);
}

void UMission::setEvent(int number)
{
  uint32_t f = 1 << number;
  eventFlags |= f;
//   if (eventEcho)
//   { // send message back to client
//     const int MSL = 20;
//     char s[MSL];
//     snprintf(s, MSL, "# event %d\n", number);
//     usb_send_str(s);
//   }
}
