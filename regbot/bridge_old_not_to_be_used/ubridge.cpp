/*
 * bridge respond functions to data changes
 * 
 ***************************************************************************
 *   Copyright (C) 2017 by DTU (Christian Andersen)                        *
 *   jca@elektro.dtu.dk                                                    *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Lesser General Public License as        *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Lesser General Public License for more details.                   *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include <string.h>
#include <unistd.h>

#include "udataitem.h"
#include "ubridge.h"
#include "uoled.h"
#include "uregbot.h"
// #include "udata.h"
#include "userverport.h"
#include "uhandler.h"

bool quitBridge = false;
bool restartBridge = false;



UBridge::UBridge(UTeensy * robot, UOled * display, UData * database, UServerPort* portServer)
{
  bot = robot;
  oled = display;
  data = database;
  serv = portServer;
  lastHbt.now();
  start();
}

UBridge::~UBridge()
{
}

void UBridge::setHandler(UHandler* messageHandler)
{
  handler = messageHandler;
}


int UBridge::getResponceNumber(const char* key, int client, bool * sequence)
{
  int responder = -1;
  if (strcmp(key, "#") == 0)
    responder = 4;
  else if (client >= 0)
  {
    const char * p1 = strchr(key, '=');
    if (strcmp(key, "robot") == 0)
      responder = 0;
    else if (strcmp(key, "oled") == 0)
      responder = 2;
    else if (strcmp(key, "client") == 0)
      responder = 5;
    else if (strcmp(key, "bridge") == 0)
      responder = 3;
    else if (strcmp(key, "help") == 0)
      responder = 3;
    else if (strcmp(key, "hbt") == 0)
      responder = 2;
    else if (strcmp(key, "u4") == 0)
      responder = 6; // get robot ID - should trigger heartbeat, if real client
    //   /**
  //    *      snprintf(reply, MRL, "#   M=N         Select mission 0=user mission, 1.. = hard coded mission (M=%d)\r\n", control.mission);
  //    *      snprintf(reply, MRL, "#   i=1         Interactive: 0: GUI info, 1: use local echo of all commands, -1:no info  (i=%d)\r\n", localEcho);
  //    *      snprintf(reply, MRL, "#   s=N A       Log interval in milliseconds (s=%d) and allow A=1 (is %d)\r\n", logInterval, logAllow);
  //    *      snprintf(reply, MRL, "#   log+/-item  Log item (mis %d, acc %d, gyro %d, " /*"mag %d, " */ "motR %d, "
  //    *      "motV %d, motA %d, enc %d, mvel %d, tr %d, pose %d, line %d, dist %d, bat %d, ct %d)\r\n",
  //    *      logRowFlags[LOG_MISSION], logRowFlags[LOG_ACC], logRowFlags[LOG_GYRO], //logRowFlags[LOG_MAG],
  //    *      logRowFlags[LOG_MOTV_REF], logRowFlags[LOG_MOTV], 
  //    *      logRowFlags[LOG_MOTA], logRowFlags[LOG_ENC], logRowFlags[LOG_WHEELVEL], logRowFlags[LOG_TURNRATE], logRowFlags[LOG_POSE], 
  //    *      logRowFlags[LOG_LINE], logRowFlags[LOG_DIST],
  //    *      logRowFlags[LOG_BATT], /*logRowFlags[LOG_BAL_CTRL],*/ logRowFlags[LOG_CTRLTIME]);
  //    *      snprintf(reply, MRL, "#   log start    Start logging to %dkB RAM (is=%d, logged %d/%d lines)\r\n", LOG_BUFFER_MAX/1000, loggerLogging(), logRowCnt, logRowsCntMax);
  //    *      snprintf(reply, MRL, "#   log get      Transfer log to USB (active=%d)\r\n", logToUSB);
  //    *      snprintf(reply, MRL, "#   motw m1 m2   Set motor PWM -1024..1024 (is=%d %d)\r\n", motorAnkerPWM[0], motorAnkerPWM[1]);
  //    *      snprintf(reply, MRL, "#   motv m1 m2   Set motor voltage -6.0 .. 6.0 (is=%.2f %.2f)\r\n", motorAnkerVoltage[0], motorAnkerVoltage[1]);
  //    *      snprintf(reply, MRL, "#   mote m1 m2   Set motor enable (left right) (is=%d %d)\r\n", motorEnable[0], motorEnable[1]);
  //    *      snprintf(reply, MRL, "#   u0..u8       Status: u0:ver,u1:measure,u2:mission, u3:log,u4:robot,u5:heartbeat,u6:mag,u8:motorVA\r\n");
  //    *      snprintf(reply, MRL, "#   u9..u14      Status: Line sensor u9=limit w, u10=limits B, u11=value, u12=raw, u13=pos and x, u14=ADC\r\n");
  //    *      snprintf(reply, MRL, "#   u15..u24     Status:  15=acc, 16=gyro, 17=gyro offs, 18=motor (A), 19=enc, 20=vel(m/s), 21=pos(m), 22=pose, 23=button, 24=voltage (V)\r\n");
  //    *      snprintf(reply, MRL, "#   u25..u28     Status:  25,26 encoder calibrate raw values (cpu clock units), u27,28 calibrate factors \r\n");
  //    *      snprintf(reply, MRL, "#   v0..2        Status: v0:IR sensor data, v1:wifi status, v2 wifi clients\r\n");
  //    *      snprintf(reply, MRL, "#   xID          Get Control params, ID: cvel=wheel vel, ctrn=turn, cwve=wall vel, cwth=wall turn, cpos=position, \r\n"
  //    *                           "#                cedg=edge, cbal=balance, cbav=bal vel, ctrl=rate limit\r\n");
  //    *      usb_send_str(        "#   ID x x ...   Set controler parameters (ID use Kp ... see format below)\r\n");
  //    *      usb_send_str(        "#         format: ID use kp iuse itau ilim Lfuse LfNum LfDen Lbuse LbNum LbDen preUse preNum preDen\r\n");
  //    *      usb_send_str(        "#                 preIuse preItau preIlim ffUse ffKp ffFuse ffFnum ffFden LimUse Lim\r\n");
  //    *      //     usb_send_str(        "#   rX=d d d ... Settingsfor regulator X - use GUI or see code for details\r\n");
  //    *      snprintf(reply, MRL, "#   rid=d d d d d d d d d d  Robot ID set: ID wheelBase gear PPR RadLeft RadRight balanceOffset batt_use batt_low HW_version\r\n");
  //    *      snprintf(reply, MRL, "#   eew          Save configuration to EE-Prom\r\n");
  //    *      snprintf(reply, MRL, "#   eeW          Get configuration as string\r\n");
  //    *      snprintf(reply, MRL, "#   eer          Read configuration from EE-Prom\r\n");
  //    *      snprintf(reply, MRL, "#   eeR=X        Read config and mission from hard coded set X=0: empty, X=1 follow wall\r\n");
  //    *      snprintf(reply, MRL, "#   sub s p m    Subscribe s=1, unsubscribe s=0, p=priority 0 (high) .. 4(low), "
  //    *                                            "m=msg:0=hbt, 1=pose,2=IR,3=edge,4=acc,5=gyro,6=amp,7=vel\r\n");
  //    *      snprintf(reply, MRL, "#   sut t p      msg interval for priority, t=time in ms 1.., p=priority 0..4 (is 0=%d 1=%d 2=%d 3=%d 4=%d)\r\n", 
  //    *               subscribe.sendInterval[0], subscribe.sendInterval[1], subscribe.sendInterval[2],
  //    *               subscribe.sendInterval[3], subscribe.sendInterval[4]);
  //    *      snprintf(reply, MRL, "#   posec        Reset pose and position\r\n");
  //    *      snprintf(reply, MRL, "#   gyroo        Make gyro offset calibration\r\n");
  //    *      snprintf(reply, MRL, "#   mem          Some memory usage info\r\n");
  //    *      snprintf(reply, MRL, "#   start        start mission now\r\n");
  //    *      snprintf(reply, MRL, "#   stop         terminate mission now\r\n");
  //    *      snprintf(reply, MRL, "#   rc=A V T     Remote control A=0/1 (is %d), V=m/s (is %g), T=vel difference (is %g)\r\n", remoteControl, remoteControlVel, remoteControlVelDif);
  //    *      snprintf(reply, MRL, "#   <add user-mission-line>     add a user mission line (%d lines loaded in %d threads)\r\n", 
  //    *               userMission.getLinesCnt(), userMission.getThreadsCnt());
  //    *      snprintf(reply, MRL, "#   <mod T L user-mission-line  Modify line L in thread T to new user-mission-line\r\n");
  //    *      snprintf(reply, MRL, "#   <clear>      Clear all user mission lines\r\n");
  //    *      snprintf(reply, MRL, "#   <get>        Get all user mission lines\r\n");
  //    *      snprintf(reply, MRL, "#   <event=X>    Make an event number X (from 0 to 31)\r\n");
  //    *      snprintf(reply, MRL, "#   <token>      Get all user mission lines as tokens\r\n");
  //    *      snprintf(reply, MRL, "#   :xxxx        Send data (AT commands) to wifi board (all except the ':') \\r\\n is added\r\n");
  //    *      snprintf(reply, MRL, "#   link=L,data  Send data to socket link L\r\n");
  //    *      snprintf(reply, MRL, "#   wifi use port SSID PW   Wifi setup (e.g. 'wifi 1 24001 \"device\" \"\"')\r\n");
  //    *      snprintf(reply, MRL, "#   wifi e/n     Echo all received from 8266 to USB link\r\n");
  //    *      snprintf(reply, MRL, "#   halt         Turn 12V power off (on by halt=0) (is %d)\r\n", batteryHalt);
  //    *      snprintf(reply, MRL, "#   alive        Alive command - reply: <alive last=\"0.0xxx\"/>\r\n");
  //    *      snprintf(reply, MRL, "#   iron 1       IR sensor on (1 = on, 0 = off) is=%d\r\n", useDistSensor);
  //    *      snprintf(reply, MRL, "#   irc 2 8 2 8 u i  IR set 20cm 80cm 20cm 80cm on installed\r\n");
  //    *      snprintf(reply, MRL, "#   servo N P V  Set servo N=1..3 (4,5) P (position):-512..+512 (>1024 = disable), V (velocity): 0=full, 1..10 P-values/per ms\r\n"
  //    *                           "#                (status: (servo 1,2,3) enabled %d,%d,%d, P= %d, %d, %d, V= %d, %d, %d)\r\n", 
  //    *               servo.servoEnabled[0], servo.servoEnabled[1], servo.servoEnabled[2],
  //    *               servo.servoValue[0], servo.servoValue[1], servo.servoValue[2],
  //    *               servo.servoVel[0], servo.servoVel[2], servo.servoVel[2] 
  //    snprintf(reply, MRL, "#   servo -1 frw Set PWM frequency\r\n");
  //    snprintf(reply, MRL, "#   svo          Get servo status (same format as below)\r\n");
  //    snprintf(reply, MRL, "#   svo s1 p1 v1 s2 p2 v2 s3 p3 v3 s4 p4 v4 s5 p5 v5  Set servo status sx=enable px=position (-1000..1000), vx velocity 0 (full) 1..10 (slower)\r\n");
  //    snprintf(reply, MRL, "#   encc, enci     Encoder calibrate: enc: calibrate now, eni: Find calibration index now\r\n");
  //    snprintf(reply, MRL, "#   eneX=1       enec=1/0 enable timing collect, eneu=1/0 use calibration, enea=1/0 auto calibrate\r\n");
  //    snprintf(reply, MRL, "#   silent=1     Should USB be silent, if no communication (1=auto silent)\r\n");
  //    snprintf(reply, MRL, "#   help         This help text\r\n");
  //    * */
    else if (strncmp(key, "start", MAX_ITEM_KEY_LENGTH) == 0 or
            strncmp(key, "stop", MAX_ITEM_KEY_LENGTH) == 0 or
            strncmp(key, "sub", MAX_ITEM_KEY_LENGTH) == 0 or
            strncmp(key, "sut", MAX_ITEM_KEY_LENGTH) == 0 or
            strncmp(key, "eew", MAX_ITEM_KEY_LENGTH) == 0 or
            strncmp(key, "eeW", MAX_ITEM_KEY_LENGTH) == 0 or
            strncmp(key, "eer", MAX_ITEM_KEY_LENGTH) == 0 or
            strncmp(key, "posec", MAX_ITEM_KEY_LENGTH) == 0 or
            strncmp(key, "gyroo", MAX_ITEM_KEY_LENGTH) == 0 or
            strncmp(key, "mem", MAX_ITEM_KEY_LENGTH) == 0 or
            strncmp(key, "<add", MAX_ITEM_KEY_LENGTH) == 0 or
            strncmp(key, "<mod", MAX_ITEM_KEY_LENGTH) == 0 or
            strncmp(key, "<clear", MAX_ITEM_KEY_LENGTH) == 0 or
            strncmp(key, "<get", MAX_ITEM_KEY_LENGTH) == 0 or
            strncmp(key, "<token", MAX_ITEM_KEY_LENGTH) == 0 or
            strncmp(key, "wifi", MAX_ITEM_KEY_LENGTH) == 0 or
            strncmp(key, "halt", MAX_ITEM_KEY_LENGTH) == 0 or
            strncmp(key, "iron", MAX_ITEM_KEY_LENGTH) == 0 or
            strncmp(key, "irc=", MAX_ITEM_KEY_LENGTH) == 0 or
            strncmp(key, "servo", MAX_ITEM_KEY_LENGTH) == 0 or
            strncmp(key, "svo", MAX_ITEM_KEY_LENGTH) == 0 or
            strncmp(key, "sv1", MAX_ITEM_KEY_LENGTH) == 0 or
            strncmp(key, "encc", MAX_ITEM_KEY_LENGTH) == 0 or
            strncmp(key, "enci", MAX_ITEM_KEY_LENGTH) == 0 or
            strncmp(key, "enec=", MAX_ITEM_KEY_LENGTH) == 0 or
            strncmp(key, "eneu=", MAX_ITEM_KEY_LENGTH) == 0 or
            strncmp(key, "enea=", MAX_ITEM_KEY_LENGTH) == 0 or
            strncmp(key, "lcl", MAX_ITEM_KEY_LENGTH) == 0 or
            strncmp(key, "M=", MAX_ITEM_KEY_LENGTH) == 0 or
            strncmp(key, "mote", MAX_ITEM_KEY_LENGTH) == 0 or
            strncmp(key, "motv", MAX_ITEM_KEY_LENGTH) == 0 or
            strncmp(key, "motw", MAX_ITEM_KEY_LENGTH) == 0 or
            strncmp(key, "rid", MAX_ITEM_KEY_LENGTH) == 0 or
            strncmp(key, "silent=", MAX_ITEM_KEY_LENGTH) == 0 or
            //             strncmp(key, "help", MAX_ITEM_KEY_LENGTH) == 0 or
            // log get or set
            strncmp(key, "log", MAX_ITEM_KEY_LENGTH) == 0 or
            strncmp(key, "log+", 4) == 0 or
            strncmp(key, "log-", 4) == 0 or
//             strncmp(key, "liv", MAX_ITEM_KEY_LENGTH) == 0 or
//             strncmp(key, "lip", MAX_ITEM_KEY_LENGTH) == 0 or
            // all keywords ending with a '='
            (p1 != NULL and p1 > key) or
            // controller param set or get
            (key[0] == 'x' and key[1]=='c' and strlen(key) == 4) or
            (key[0] == 'c' and strlen(key) == 3) or
            // request message
            (key[0] == 'u' and isdigit(key[1])) or
            (key[0] == 'v' and isdigit(key[1])) 
    )
      responder = 1;
    else
    {
      printf("URespond::getResponceNumber: no responder for '%s' (from client=%d)\n", key, client);
    }
    if (responder >= 0)
      *sequence = true;
  }
  else if (client == CLIENT_ROBOT)
  {
    if (strncmp(key, "<m", MAX_ITEM_KEY_LENGTH) == 0 or
        strncmp(key, "log", MAX_ITEM_KEY_LENGTH) == 0 or
        strncmp(key, "event", MAX_ITEM_KEY_LENGTH) == 0 or
        false
         )
    { // important message sequence - mission and log
      *sequence = true;
      printf("URespond::getResponceNumber: '%s' set as sequence (from client=%d)\n", key, client);
    }
    else if (strncmp(key, "hbt", MAX_ITEM_KEY_LENGTH) == 0)
    {
      responder = 2;
    }
  }
  else if (client == CLIENT_JOY)
  { // here are messages to handle too
    if (strncmp(key, "rc=", MAX_ITEM_KEY_LENGTH) == 0)
    {
      responder = 1;
    }
  }
//   printf("UBridge::responder: key=%s get responder=%d\n", key, responder);
  return responder;
}


void UBridge::responce(int responceNumber, UDataItem* dataItem, int client)
{
  switch(responceNumber)
  {
    case 0: responceParamToRobot(dataItem, client); break;
    case 1: responceAllToRobot(dataItem); break;
    case 2: responceOled(dataItem); break;
    case 3: responceBridge(dataItem, client); break;
    case 4: responceConsole(dataItem, client); break;
    case 5: responceClient(dataItem, client); break;
    case 6: responceRobotID(dataItem, client); break;
    default:
      printf("UBridge::responce: unknown responce handler\n");
      break;
  }
}


void UBridge::responseParamToRobot(UDataItem* dataItem, int client)
{ // responder - relay message parameters only to robot
  const char * p1 = dataItem->itemParams.c_str();
  if (strncmp(p1, "help", 4) == 0)
  { // set client to het help messages from robot
    // sideeffect is that client also gets all other messages
    handler->handleCommand(client, (char*)"# subscribe 6", true);
    serv->sendString("\r\n# NB! you now subscribed to all '#' messages\r\n"
                         "# (use 'robot silent' or '# subscribe 0' to undo)\r\n", 
                         client);
    const char * help = "# Robot connection has these subcommand options:\r\n"
    "#     clogopen   open communication log for all traffic to and from Regbot\r\n"
    "#     clogclose  close these logfiles\r\n"
    "#     help       This help text\r\n";
    serv->sendString(help, client);
    const int MSL = 100;
    char s[MSL];
    snprintf(s, MSL, 
    "#     Regbot communication open=%d\r\n", bot->regbotConnectionOpen);
    serv->sendString(s, client);
    
  }
  if (strncmp(p1, "silent", 6) == 0)
  {
    handler->handleCommand(client, (char*)"# subscribe 0", true);
    printf("# got a silent -> # subscribe 0\n");
  }
  else if (strncmp(p1, "clogopen", 7) == 0)
  {
    bot->openCommLog(data->logPath);
    serv->sendString("# opend robot communication logfiles\n", client);
//     printf("# got a clogopen -> opens 2 logfiles\n");
  }
  else if (strncmp(p1, "clogclose", 7) == 0)
  {
    bot->closeCommLog();
    serv->sendString("# closed robot communication logfiles\n", client);
//     printf("# got a cloclose -> closes 2 logfiles\n");
  }
  else
  { // not silent, so forward to robot
    bot->send(dataItem->itemParams.c_str(), "", 10);
//     printf("# got other: robot %s\n", p1);
  }
}


void UBridge::responceAllToRobot(UDataItem* dataItem)
{ // responder - relay message parameters only to robot
  bot->send(dataItem->itemKey, dataItem->itemParams.c_str(), 10);
}

float UBridge::measureCPUtemp()
{
  FILE * temp;
  const char * tf = "/sys/class/thermal/thermal_zone0/temp";
  temp = fopen(tf, "r");
  float t = 0;
  if (temp != NULL)
  {
    const int MSL = 20;
    char s[MSL];
    char * p1 = s;
    int n = fread(p1, 1, 1, temp);
    int m = n;
    while (n > 0)
    {
      n = fread(++p1, 1, 1, temp);
      m += n;
    }
    s[m] = '\0';
    if (m > 3)
    {
      t = strtof(s, &p1);
    }
//     printf("#got %d bytes (%g deg) as:%s\n", m, t/1000.0, s); 
    fclose(temp);
  }
  return t/1000.0;
}


void UBridge::responceStatus(UDataItem * dataItem)
{
  const char * p1 = dataItem->itemParams.c_str();
  if (strncmp(dataItem->itemKey, "hbt", 3) == 0)
  {
    char * p2 = (char *)p1;
    float t = strtof(p2, &p2);
    float v = strtof(p2, &p2);
    if (p2 != NULL and *p2 != '\0')
    { // there is more
      bool ctrl = strtol(p2, &p2, 10);
      bot->missionState  = strtol(p2, &p2, 10);
      bool rc   = strtol(p2, &p2, 10);
      if (false)
        printf("UBridge::responcrOled: ctrl=%d, Rc=%d, mis=%d, par='%s'\r\n", ctrl, rc, bot->missionState, p1);
    }
    if (false)
      printf("UBridge::responcrOled: hbt v=%f, t=%f\r\n", v, t);
    oled->displayVoltageAndName(v, t, measureCPUtemp());
    //
    lastHbt.now();
  }
  else if (strncmp(p1, "clear", 5) == 0)
  { // reset display to default
    oled->clear();
  }
  else
  {
    char * p2 = (char *)p1;
    int n = strtol(p1, &p2, 10);
    if (n < 0 or n > 7)
    { // not legal - use default line
      n = 3;
      printf("UBridge::responcrOled: illegal line number - print on line 3\r\n");
    }
    while (*p2 <= ' ' and *p2 != '\0')
      p2++;
    if (strlen(p2) > 0)
    {
      oled->printLine(n, p2);
      // display is used for status - don't continue with IP
      oled->displayIP = false;
    }
    else
      printf("URespond::responcrOled: empty string - no display\r\n");
  }
}



void UBridge::responceBridge(UDataItem* dataItem, int client)
{
  const char * p1 = dataItem->itemParams.c_str();
  if (strncmp(p1, "list", 4) == 0)
  {
    listItemIterator = 0;
    listItems = true;
    listClient = client;
  }
  else if (strncmp(p1, "restart", 4) == 0)
  {
    restartBridge = true;
    quitBridge = true;
  }
  else if (strncmp(p1, "quit", 4) == 0)
  {
    quitBridge = true;
  }
  else if (strncmp(p1, "help", 4) == 0)
  { // bridge help
    const char * help = "# Bridge has these subcommand options:\r\n"
                        "#     list    Gives a list of static information on all data items\r\n"
                        "#     restart Restart the bridge (implementing newly compiled version)\r\n"
                        "#     quit    stop bridge\r\n"
                        "#     help    This help text\r\n";
    serv->sendString(help, client);
  }
  // also data item where key="help"
  if (strcmp(dataItem->itemKey, "help") == 0)
  { // overall help for bridge
    const char * help = "# Robobot_bridge help:\r\n"
    "# Commands are all single line text staring with a keyword\r\n"
    "#     (up to 6 characters) followed by text or numbers:\r\n"
    "# Main topics:\r\n"
    "#     robot       Sends all rest of the line to the robot, see 'robot help' for options\r\n"
    "#     bridge      See 'bridge help' for options\r\n"
    "#     oled L xxx  Prints xxx (up to 20 chars) on line L (2..7) on oled\r\n"
    "#     client      See 'client help' for options\r\n"
    "#     q           quit\r\n"
    "#     h           Console help\r\n"
    "#     help        This help\r\n"
    "# All lines starting with '#' are assumed to be comments.\r\n"
    "# All commands are stored as a data item with a keyword as ID.\r\n"
    "# All data items has reserved subcommands, see:\r\n"
    "#     item h      Help for data item 'item'.\r\n";
    serv->sendString(help, client);
  }
}

/**
 * This is a thread that handles all publish data items as needed */
void UBridge::run()
{
  while (not th1stop)
  {
    usleep(100);
    if (listItems)
    {
      if (listItemIterator >= data->getItemCnt())
      {
        listItems = false;
        listItemIterator = 0;
      }
      else
      {
        data->publishItemMeta(listItemIterator++, listClient);
      }
    }
  }
}

void UBridge::responceConsole(UDataItem* dataItem, int client)
{
  if (false)
    // never do this, unless for debug - use subscribe instead
    printf("%s %s\n", dataItem->itemKey, dataItem->itemParams.c_str());
}

void UBridge::responceClient(UDataItem* dataItem, int client)
{
  const char * p1 = dataItem->itemParams.c_str();
  if (strncmp(p1, "list", 4) == 0)
  {
    const int MSL = 1000;
    char s[MSL];
    // send this clients number first
    snprintf(s, MSL, "# Client list requested from client %d\r\n", client);
    serv->sendString(s, client);
    // make client list text into string buffer
    serv->printStatus(s, MSL);
    serv->sendString(s, client);
  }
  else if (strncmp(p1, "help", 4) == 0)
  {
    const char * help = "# Socket server help:\r\n"
    "#     list: Gives a list of active clients\r\n"
    "#     help: This help text\r\n";
    serv->sendString(help, client);
  }
}


void UBridge::responceRobotID(UDataItem* dataItem, int client)
{
//   if (client != CLIENT_CONSOLE)
//     handler->handleCommand(client, (char*)"hbt subscribe 2");
  responceAllToRobot(dataItem);
  if (strcmp(dataItem->itemKey, "u4") == 0)
  { // if robot is dead, then allow bridge to provide a heartbeat
    if (false and lastHbt.getTimePassed() > 0.8)
    { // keep client alive
      const int MSL = 100;
      char s[MSL];
      oled->oldRegbotTime -= 0.8;
      // hbt time battery ctrl mission rc\n
      snprintf(s, MSL, "hbt %.1f %.1f 0 9 0\n", oled->oldRegbotTime, oled->oldBatteryVoltage);
      serv->sendString(s, client);
    }
  }  
}
