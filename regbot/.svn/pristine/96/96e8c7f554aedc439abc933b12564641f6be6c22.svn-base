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


#include "controlbase.h"


////////////////////////////////////////////////////////////

UTransferFunctionPZ::UTransferFunctionPZ()
{
  sampleTime = 0.001;
  inUse = false;
}


void UTransferFunctionPZ::setParamsTauTau(float tau_numerator, float tau_denominator, float out_limit)
{
  tau_num = tau_numerator;
  tau_den = tau_denominator;
  inUse = out_limit <= 0.0;
  limit = out_limit;
  output_limit = out_limit < 1e6 and out_limit > 0.0;
}

////////////////////////////////////////////////////////////

void UTransferFunctionPZ::initControl()
{ // include pole zero init
  if (inUse)
  { /* Pole - Zero 1. order filter 
    * Gd:=(td * s + 1)/(al*td*s + 1);
    * denominator (nævner)
    * Gzd2:=expand(Gzd*z^(-1));
    *
    *      (2*al*td + T)u + (T- 2*al*td) u/z
    *      tau_den = td*al;
    * numerator (tæller)
    * Gzn2:=expand(Gzn*z^(-1));
    *                                 
    *      (T + 2*td)e + (T - 2*td) e/z
    *      tau_num = td
    *
    * controller code: u[0] =  -u[1]*zu[1] + e*ze[0] + e[1] * ze[1];
    */
    zu[0] = 2 * tau_den + sampleTime; // denominator 
    zu[1] = (sampleTime - 2 * tau_den) / zu[0];
    ze[0] = (sampleTime + 2 * tau_num) / zu[0]; // numerator
    ze[1] = (sampleTime - 2 * tau_num) / zu[0];
  }
  else
  { // no function (transfer = 0)
    zu[0] = 1;
    zu[1] = 1; // subtract old value - if any
    ze[0] = 0; // do not use input
    ze[1] = 0;
  }
}

////////////////////////////////////////////////////////////

/**
 * simple filter 1. order with output limit*/
void UTransferFunctionPZ::controlTick()
{
  if (inUse)
  {
    y[0] = -zu[1] * y[1] + ze[0] * x[0] + ze[1] * x[1];
    if (output_limit)
    {
      if (y[0] > limit)
        y[0] = limit;
      else if (y[0] < -limit)
        y[0] = -limit;
    }
    y[1] = y[0];
    x[1] = x[0];
  }
  else
    y[0] = 0.0;
}


const char * UTransferFunctionPZ::decodeFromString(const char * line)
{
  const char * p1 = line;
  char * p2;
  inUse = strtol(p1, &p2, 0);
  if (p2 > p1)
  {
    p1 = p2;
    tau_num = strtof(p1, &p2);
  }
  if (p2 > p1) 
  {
    p1 = p2;
    tau_den = strtof(p1, &p2);
  }
  if (p2 > p1)
    return p2;
  else
    return NULL;
}

////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////

UTransferFunctionI::UTransferFunctionI()
{
  sampleTime = 0.001;
  inUse = false;
}

////////////////////////////////////////////////////////////

void UTransferFunctionI::setParamsIntegrator(float tau, float i_limit)
{
  tau_num = 1.0;
  tau_den = tau;
//   filter_use = i_limit <= 0.0;
  limit = i_limit;
  output_limit = i_limit < 1e6 and i_limit > 0.0;
}

////////////////////////////////////////////////////////////

const char * UTransferFunctionI::decodeFromString(const char * line)
{ // from e.g. "1 0.22 4.77"
  const char * p1 = line;
  const char * p2 = p1;
  inUse = strtol(p1, (char**)&p2, 0);
  if (p2 > p1)
  {
    p1 = p2;
    tau_num = strtof(p1, (char**)&p2);
  }
  if (p2 > p1) 
  {
    p1 = p2;
    limit = strtof(p1, (char**)&p2);
    if (limit < 1e6 and limit >= 0.0)
      output_limit = true;
  }
  if (p2 > p1)
    return p2;
  else
    return NULL;
}

////////////////////////////////////////////////////////////

void UTransferFunctionI::initControl()
{ // include pole zero init
  if (inUse)
  { // first order integrator
    /*
     * GC(s) = u/e = 1/(tau_i s)
     * Gzd2:=expand(Gzd*z^(-1));
     *                        
     *    (2*ti)u + (2*ti)u/z
     *                        
     * Gzn2:=expand(Gzn*z^(-1));
     *                
     *    T*e + T*e/z 
     *                
     *  u[0] = u[1] + T/(2*ti) * e[0] + T/(2*ti) * e[1]
     * 
     */
    zu[0] = 2 * tau_den; // denominator
    zu[1] = -1;
    ze[0] = sampleTime / zu[0]; // numerator
    ze[1] = sampleTime / zu[0];
  }
  else
  { // no added value from integrator
    zu[0] = 1;
    zu[1] = 1;
    ze[0] = 0;
    ze[1] = 0;
  }
}

////////////////////////////////////////////////////////////

/**
 * simple filter 1. order */
void UTransferFunctionI::controlTick()
{
  if (inUse)
  {
    y[0] = -zu[1] * y[1] + ze[0] * x[0] + ze[1] * x[1];
    if (output_limit)
    {
      if (y[0] > limit)
        y[0] = limit;
      else if (y[0] < -limit)
        y[0] = -limit;
    }
    y[1] = y[0];
    x[1] = x[0];
  }
  else
    y[0] = 0.0;
}


////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////


UControlBase::UControlBase()
{
  use = false;
  kp = 1;
  ffKp = 0.0;
  rateLimitUse = false;
  outLimitUse = false;
}

////////////////////////////////////////////////////////////////////////

bool UControlBase::initRegulator(const char * line)
{
  const char * p1 = line;
  char * p2;
  bool isOK = false;
  use = strtol(p1, &p2, 10);
  if (p2 > p1)
  {
    p1 = p2;
    kp = strtof(p1, &p2);
  }
  if (p2 > p1)
  {
    p1 = integrator.decodeFromString(p2);
    if (p1 != NULL)
      p1 = leadFwd.decodeFromString(p1);    
    if (p1 != NULL)
      p1 = leadBack.decodeFromString(p1);    
    if (p1 != NULL)
      p1 = preFilt.decodeFromString(p1);    
    if (p1 != NULL)
      p1 = preFiltI.decodeFromString(p1);    
    if (p1 != NULL)
      ffUse = strtol(p1, &p2, 0);
    if (p1 != NULL and p2 > p1)
      ffKp = strtof(p1 = p2, &p2);
    if (p1 != NULL and p2 > p1)
      p1 = ffFilt.decodeFromString(p1 = p2);    
    if (p1 != NULL)
      outLimitUse = strtol(p1, &p2, 0);
    if (p1 != NULL and p2 > p1)
      outLimit = strtof(p1 = p2, &p2);
    if (p1 != NULL and p2 > p1)
      rateLimitUse = strtol(p1 = p2, &p2, 0);
    if (p1 != NULL and p2 > p1)
    {
      rateLimit = strtof(p1 = p2, &p2);
      isOK = true; 
    }
  }
  return isOK;
}


void UControlBase::controlTick()
{
//   if (false)
//     if (mission_wall_turn)
//     { // brug kortest afstand
//       if (irDistance[0]  < irDistance[1] * 0.7)
//         m = irDistance[0];
//       else
//         m = irDistance[1] * 0.65;
//     }
//     else
//       // sensor looking forward, 
//       m = irDistance[1];
//     if (missionState > 1)
//     {
//       regulWall(mission_wall_ref, m);
//       if (m > 0.5)
//         // too far away to trust for control
//         regWallU[0] = 0;
//     }
//     else
//       // run straight until line - or something
//       regWallU[0] = 0;
//     //
//     if (mission_wall_turn)
//     { // we are controlling turn
//       if (regWallU[0] > 0.0)
//       { // reduce inner wheel only
//         regul_turn_vel_reduc[0] = regWallU[0];
//         regul_turn_vel_reduc[1] = 0;
//       }
//       else
//       {
//         regul_turn_vel_reduc[0] = 0;
//         regul_turn_vel_reduc[1] = -regWallU[0];
//       }
//       mission_wall_vel_ref = 0;
//     }
//     else 
//     { // 
//       mission_vel_ref = 0;
//       mission_wall_vel_ref = -regWallU[0];
//     }
//   }
//   else
//   { // no mission active - reset controller
//     mission_wall_ref = 0.0;
//     regWallE[1] = 0;
//     regWallUD[1] = 0;
//     regWallUI[1] = 0;
//     if (regul_wall_do_turn)
//     {
//       regul_turn_vel_reduc[0] = 0.0;
//       regul_turn_vel_reduc[1] = 0.0;
//     }
//     else
//       mission_wall_vel_ref = 0;
//   }
}
