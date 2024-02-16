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
 ***************************************************************************/

#ifndef CONTROL_BASE_H
#define CONTROL_BASE_H

#include <string.h>
#include <stdlib.h>
#include "WProgram.h"

class UTransferFunctionPZ
{
public:
  /**
   * constructor */
  UTransferFunctionPZ();
  /**
   * Init discrete control parameters */
  void initControl();
  /**
   * Set parameters */
  void setParamsTauTau(float tau_numerator, float tau_denominator, float limit);
  /**
   * decode from 3 values in a string 
   * \param line format is "1 0.22 0.022" for "use=true tauNumerator=0.22 tauDenominator=0.022"
   * \returns pointer to next unused character on line, or NULL if format error */
  const char * decodeFromString(const char * line);
  /**
   * Set parameters */
//   void setParamsTauAlpha(float tau, float alpha, float limit);
  /**
   * Do calculate control */
  void controlTick();
public:
  bool inUse;
protected:
  float tau_num;
  float tau_den;
  float sampleTime;
  float limit;
//   bool filter_use;
  bool output_limit;
private:
  /** control factor for numerator */
  float ze[2];
  /** control factor for denominator */
  float zu[2];
  /** current control input values [0]=current [1]=last value */
  float x[2];
  /** current control output values [0]=current [1]=last value */
  float y[2];
};

class UTransferFunctionI
{
public:
  /**
   * constructor */
  UTransferFunctionI();
  /**
   * Init discrete control parameters */
  void initControl();
  /**
   * Set parameters */
  void setParamsIntegrator(float tau, float limit);
  /**
   * decode from 3 values in a string 
   * \param line format is "1 0.22 4.5" for "use=true tau=0.22 limit=4.5"
   * \returns pointer to next unused character on line, or NULL if format error */
  const char * decodeFromString(const char * line);
  /**
   * Do calculate control */
  void controlTick();
public:
  bool inUse;
protected:
  float tau_num;
  float tau_den;
  float sampleTime;
  float limit;
//   bool filter_use;
  bool output_limit;
private:
  /** control factor for numerator */
  float ze[2];
  /** control factor for denominator */
  float zu[2];
  /** current control input values [0]=current [1]=last value */
  float x[2];
  /** current control output values [0]=current [1]=last value */
  float y[2];
};

///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

class UControlBase
{
public:
  /**
   * constructor */
  UControlBase();
  /**
  * init regulator from string 
  * \param line is a formatted string:
  *     "rxx use kp  ui ti iLimit  u1 t1n t1d  u2 t2n t2d   u3 t3n t3d u3i t3i lim3  ...
  *          ffuse kp4 u4 t4n t4d  lim_use limit rate_use rate_limit"
  * where the part in quotes are the line parameter, and:
  * use is controller active, else factor one from in to out
  * kp is proportional gain
  * ui ti iLimit is integrator use, time constant, and i limit (1e10 is no limit)
  * u1 t1n, t1d is Lead fwd: use, numerator,denominator time constants in  (t1n*s + 1)/(t1d*s + 1)
  * u2 t2n, t2d is Lead back use, numerator,denominator time constants in  (t2n*s + 1)/(t2d*s + 1) 
  * u3 t3n, t3d is pre-filter use, numerator,denominator time constants in  (t3n*s + 1)/(t3d*s + 1)
  * u3i t3i, lim3 is pre-filter i_term: use, tau_i, and limit
  * ffuse kp4 is feed forward use and proportional gain 
  * u4 t4n, t4d is feed forward use, numerator,denominator time constants in  (t4n*s + 1)/(t4d*s + 1)
  * u_limit is output limit      > 1e10 marks do not use
  * rate_limit for reference value >1e10 marks do not use
  * \returns true if all valid parameters are found */
   bool initRegulator(const char * line);
   /**
    * control tick 
    * This will calculate the full controller. */
   void controlTick();
protected:
  /** use regulator, if false, then reference is send to output directly */
  bool use;
  bool rateLimitUse;
  float rateLimit;
  float kp;
  bool ffUse;
  float ffKp;
  bool outLimitUse;
  float outLimit;
private:
  UTransferFunctionPZ preFilt;
  UTransferFunctionI preFiltI;
  UTransferFunctionPZ ffFilt;
  UTransferFunctionI integrator;
  UTransferFunctionPZ leadFwd;
  UTransferFunctionPZ leadBack;
};

#endif