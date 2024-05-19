/*
  main.cpp - Main projectfile to run OPAL FW on PJRC Teensy 4.x board

  Part of OpenGalvo - OPAL Firmware

  Copyright (c) 2020-2021 Daniel Olsson

  OPAL Firmware is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  OPAL Firmware is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with OPAL Firmware.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "main.h"
#include <Synrad48Ctrl.h>


static CircularBuffer<GCode, BUFFERSIZE> commandBuffer;
static CircularBuffer<String, BUFFERSIZE> FWDBuffer;

MotionMGR* motion;

XY2_100* galvo;

SerialCMDReader *serialReciever;

#ifdef LASER_IS_SYNRAD
  //Synrad48Ctrl syn;
#endif

LaserController *laser;
void setup() {
  serialReciever = new SerialCMDReader(&commandBuffer);
  serialReciever->begin();

  // Init pins
  pinMode(LASER_SSR_OUT_PIN, OUTPUT);
  digitalWrite(LASER_SSR_OUT_PIN,0);

  pinMode(GALVO_SSR_OUT_PIN, OUTPUT);
  digitalWrite(GALVO_SSR_OUT_PIN,0);

  //#ifdef LASER_IS_SYNRAD{
    //laser = new Synrad48Ctrl();
    //laser->begin(LASER_PWM_OUT_PIN, LASER_SSR_OUT_PIN);
  //} 
  //#endif
  #ifdef LASER_IS_FIBER{
    laser = new FiberCtrl();
    laser->begin(SHReg_Latch_PIN, Laser_Enable);
  }
  #else
    //implement PWMLaser
  #endif
  //init Galvo Protocol
  //i changed noting here but it throws errors O.O
  galvo = new XY2_100();
  galvo->begin(); //TODO:ADD define "Galvo has SSR" for galvo PSU

  motion = new MotionMGR(&commandBuffer);
  motion->begin(galvo, laser);
  printWelcome();
  Serial5.begin(115200);

  Serial5.print("G28\n");
}

char* nextFWDMSG[150];
void loop() {  
  if(Serial5.available())
  {
    ReadSerial5();
  }
  else
  {
    if(!FWDBuffer.isEmpty())
    {
      String data = FWDBuffer.pop();
      Serial5.print(data);
      Serial5.print("\n");
    }
  }
  serialReciever->handleSerial();
  motion->tic();

}

void setGalvoPosition(double x, double y)
{
  int tmp_x, tmp_y;
  if(AXIS_INVERSE_X)
    tmp_x = map(x, 0.0,X_MAX_POS_MM, 65535,0)+0.5;
  else
    tmp_x = map(x, 0.0,X_MAX_POS_MM, 0,65535)+0.5;

  if(AXIS_INVERSE_Y)
    tmp_y = map(y, 0.0,Y_MAX_POS_MM, 65535,0)+0.5;
  else
    tmp_y = map(y, 0.0,Y_MAX_POS_MM, 0,65535)+0.5;

  galvo->setPos(tmp_x, tmp_y);
}

void setLaserPower(double PWM)
{
  double tmp_MAX = 50; //not sure about this but ill see soon enough
  double pinVal = map(PWM,0.0,tmp_MAX,0,255);
  laser->update((int)pinVal);
}

void setNextFWDMSG(char MSG[150])
{
  String str = String(MSG);
  FWDBuffer.unshift(str);
}

bool ReadSerial5()
{
  bool retval = false;
  if (Serial5.available()) {
    retval = true;
    static char worda[COMMAND_SIZE], *pSdata=worda;
    byte ch;

    ch = Serial5.read();
    //mcnt++;
    // -1 for null terminator space
    if ((pSdata - worda)>=COMMAND_SIZE-1) {
        pSdata--;
        Serial.print("Serial5: BUFFER OVERRUN\n");
    }

    *pSdata++ = (char)ch;
    if (ch=='\n' || ((pSdata - worda)>=COMMAND_SIZE-3))// Command received and ready.
    {
      
      pSdata = worda;
      Serial.print("\nECHO Serial5: ");Serial.println(worda);
      xinit_process_string(worda);
    }
  }
  return retval;
}

/*
Used by Serial5 to clear input string array.
*/
void xinit_process_string(char instruction[])  {
  //init our command
  for (byte i=0; i<COMMAND_SIZE; i++)
    instruction[i] = 0;
  //mcnt = 0;
}