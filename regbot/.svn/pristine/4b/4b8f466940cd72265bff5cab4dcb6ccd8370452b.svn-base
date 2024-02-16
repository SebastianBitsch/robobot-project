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

#ifndef REGBOT_MISSION_H
#define REGBOT_MISSION_H

#include <string.h>
#include <stdlib.h>
#include "WProgram.h"

// void eePromSaveMission();
// void eePromLoadMission();

class UMissionLine;
class UMission;

const int miLinesCntMax = 50;
extern UMissionLine miLines[];
//extern int miLinesCnt;

const int missionErrStrMaxCnt = 50;
extern char missionErrStr[missionErrStrMaxCnt];

extern UMission userMission;

///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

class UMissionThread
{
  friend class UMissionLine;
public:
  int8_t misLineNum = 0;     // current mission line number
  //int misLineNumLast = 0;     // last line number to detect change
  int misStartTime;           // start time for this line
  bool turnEndedAtEndAngle;  // true if turn finished at desired angle
  float linStartDist;         // start distance for this line
  float linStartAngle;        // start heading (ref) for this line
  // thread number
  uint16_t threadNr;
  uint8_t lineCnt;
  uint8_t lineStartIndex;
protected:  
  UMissionLine * currentLine;
  uint16_t gotoLabel; // set to label number, otherwise 0.
  bool theEnd;
  float turnAngSum;            // turned angle for this line
  float turnSumLast;          // last angle for summing
public:
  void clear(uint8_t idx);
  /**
   * Test finished and advance to next line 
   * \param label is a label number, if supposed to go to a label line.
   * 
   * \returns true if line is valid (i.e. not finished with mission) */
  bool advanceLine(int16_t toLabel);
  /**
   * Test if current line is finished, and termiate any line specific actions (i.e. turn).
   * \retuns true if line is finished. */
  bool testFinished();
  /**
   * Implement new line, this advance to next line and implement 
   * any new line specific items.
   * \returns true if no more lines area available */
  bool moveToNextLine();
  /**
   * reset thread to start a new mission
   * resets line number and implements first line in thread */
  void startMission();
  /**
   * initialize new line
   * \returns true if no errors */
  void implementNewLine();
  /**
   * Reset visit counter on all lines */
  void resetVisits();
  /**
   * Get lines in this thread as readable syntax to client. */
  void getLines();
  /** 
   * Get lines as tokens (for debug) for this thread */
  void getToken();
  /**
   * Get the thread number as a token line */
  int toTokenString(char * bf, int bfCnt);
  /**
   * Add a line to this thread, if the line holds a thread keyword, 
   * that do not match the thread number and is not the first line in the thread,
   * then do not add, but return false and the thread number in the newThreadNumber.
   * \param lineToAdd is the line in normal syntax.
   * \param newThreadNumber should return the thread number, if such keyword exist
   * \returns true, if the line is added. */
  bool addLine(const char * lineToAdd, int16_t * newThreadNumber);
  /**
   * Decode this line of tokens.
   * \param line is the line, it is ended with a new-line, but not a zero.
   * \param tn should be set to the thread-number, if line line is a thread-number token.
   * \returns true if line is added, returns false if thread-number do not match current thread. */
  bool decodeToken(char * line, uint16_t * tn);
  /**
   * modify specific line
   * NB! this modifies the line even if a mission is in progress.
   * \param line is line number in this thread to modify - first line has number 1
   * \returns true is line exist (and is modified), else false. */
  bool modLine(int16_t line, const char * p2);
  
  
};

///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

class UMissionLine
{
public:
  enum MPTYP {MP_VEL='a', 
    MP_ACC, 
    MP_TR, 
    MP_LOG, 
    MP_BAL='f', // skip e, as it can be seen as exponent 7e1 = 70
    MP_EDGE_L, 
    MP_EDGE_R, 
    MP_EDGE_WHITE, 
    MP_LABEL, 
    MP_DRIVE_DIST,  /// set destination distance (driven distance) using position controller 
    MP_IR_SENSOR, 
    MP_IR_DIST,  /// use distance sensor
    MP_GOTO,     /// goto destination
    MP_THREAD,   /// thread keyword to start new thread
    MP_EVENT,    /// trigger an event
    MP_HEADING  /// set heading reference directly
  };
private:
  enum MPTYC {MC_DIST='A', 
    MC_TIME, 
    MC_TURN, 
    MC_COUNT, 
    MC_XINGB = 'F',  // skip E  as 7E1 = 70 
    MC_XINGW, 
    MC_LINE_VALID, 
    MC_IR_DIST1, /// distance sensor 1 limit
    MC_IR_DIST2, /// distance sensor 2 limit
    MC_TILT, /// tilt angle
    MC_EVENT, /// event test
    MC_LOG    /// test for log-full
  };
public:
  // drive parameters
  float vel;
  bool velUse;
  float acc;
  bool accUse;
  float log;
  bool logUse;
  bool bal;
  bool balUse;
  float tr;
  bool trUse;
  float edgeRef;
  bool edgeLUse;
  bool edgeRUse;
  bool edgeWhiteUse;
  bool edgeWhite;
  uint16_t label;
  uint16_t gotoDest;
  bool gotoUse;
  int8_t irSensor;
  int8_t irSensorUse; // 0=false, positive (1 0r 2) use sensor for turning, negative (-1or -2)use sensor for velocity control
  float irDistRef; // reference distance for ir sensor
  bool irDistRefUse;
  float drivePos;    // position controller should keep this position referencd (on this leg)
  bool drivePosUse;  // should position controller be used
  /// continue conditions
  float dist;     // distance check distance
  char distUse;   // should distance check be used
  float turn; // desired turn angle in degrees as a condition
  char turnUse; 
  float time;
  char timeUse;
  char xingWhiteUse;
  char xingBlackUse;
  int8_t xingBlackVal;
  int8_t xingWhiteVal;
  char lineValidUse;
  char lineValidVal;  
  uint16_t count;
  char countUse;
  float irDist1;
  char irDist1Use;
  float irDist2;
  char irDist2Use;
  float tilt;
  char tiltUse;
  uint32_t eventSet;
  uint32_t eventMask;
  bool logFullUse;
public:
  bool valid; // no error found
  int visits; // number of times this line has beed executed
  
public:
  /** clear all valid flags */
  void clear();
  /** convert to string - for return message 
   * \param bf is a C char array 
   * \param bfCnt is the count of bytes in bf
   * \param frame adds linefeed if true
   * \returns buffer (bf) filled from class and the used number of bytes */
  int toString(char * bf, int bfCnt, bool frame = true);
  /** decode mission line 
   * \param buffer is expected to hold a mission line in a terminated C string
   * \param threadNumber is set to the new thread number - if this keyword is on the line.
   * \returns true if loaded with no errors */
  bool decodeLine(const char * buffer, int16_t * threadNumber);
  /** convert to token string - for save in ee-prom 
   * \param bf is a C char array (destination)
   * \param bfCnt is the count of bytes in bf
   * \returns buffer (bf) filled from class and the used number of bytes */
  int toTokenString(char* bf, int bfCnt);
  /** decode mission line from token string 
   * \param buffer is expected to hold a mission line as tokens in a terminated C string
   * \returns true if loaded with no errors */
  bool decodeToken(const char* buffer);
  /**
   * Is this line finished - i.e. ready to continue to next line
   * \param state is the state of this line so far.
   * \param labelNum is 0 if not going to a label, else the label number (labels are >= 1)
   * \param endedAtEndAngle will be set to true, if line finished at desired angle
   * \returns true if finished */
  bool finished(UMissionThread * state, uint16_t * labelNum, bool * endedAtEndAngle);
  /**
   * Set values from this line, called at start of the line */
  void implementLine();
  /**
   * Postprocess this line, i.e.
   * terminate any special drive mode (turn, follow wall etc. implemented by this
   * line and not supposed to be continued on next line.
   * \param lineStartAngle is the heading when this line were started (used for turn a specific number of degrees.
   * \param endAtAngle true turn end at desired angle, false if turn ended for other reasons
   * */
  void postProcess(float lineStartAngle, bool endAtAngle);
  
};


///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

class UMission 
{
protected:
  // mission line pool
  //UMissionLine miLines[miLinesCntMax];
  // number of used lines.
  //uint8_t mLinesCnt;
  // number of threads
  static const int threadsMax = 5;
  // thread start list
  UMissionThread threads[threadsMax];
  // number of threads used
  uint8_t threadsCnt;
public:
  // event flags
  uint32_t eventFlags;
public:
  /// constructor - initialize mission
  UMission()
  {
    clear();
  };
  /** zero all mission lines. */
  void clear();
  /**
   * Test all threads for a finished mission
   * \returns true if mission is finished */
  bool testFinished();
  /**
   *(re)start current mission */
  bool startMission();
  /**
   * save all mission lines to EE-prom */
  void eePromSaveMission();
  /**
   * Load mission from ee-prom */
  void eePromLoadMission();
  /**
   * Add a mission line to default thread 
   * This is not legal if a mision is in progress 
   * \param lineToAdd is a string in clear text added to the active (latest) thread.
   * \returns true if no syntax error is found. */
  bool addLine(const char * lineToAdd);
  /**
   * modify specific line
   * NB! this modifies the line even if a mission is in progress.
   * \param thread is thread number to modify (0 to 32767)
   * \param line is line number in this thread
   * \returns true is line exist (and is modified), else false. */
  bool modLine(int16_t thread, int16_t line, const char * p2);
  /**
   * Send all mission lines to console (USB or serial) */
  void getLines();
  /**
   * Get the packed token string to client - for debug */
  void getToken();
  /**
   * Get number of threads */
  inline int getThreadsCnt()
  { return threadsCnt; };
  /**
   * Get number of lines */
  int getLinesCnt();
  /**
   * Get thread index number.
   * \param threadNumber is the thread number to look for,
   * \returns the index of the thread, or -1 if not found. */
  int16_t getThreadIndex(int16_t thread);
  /**
   * Set an event flag 
   * \param event number in range 0..31 */
  void setEvent(int number);
  /**
   * Decode event number and activate this event.
   * \param string from client, everything after keyword 'event'
   * assumed to be '=12\ n' or similar */
  void decodeEvent(const char * eventNumber);
  
  
};

#endif