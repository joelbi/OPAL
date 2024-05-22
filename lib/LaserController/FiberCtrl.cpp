/*
  Synrad48Ctrl.h - driver code for Synrad 48 Series laser on PJRC Teensy 4.x board

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

#include <Arduino.h>

//#ifndef TEENSY_TIMER_TOOL
//#include <TeensyTimerTool.h>
//using namespace TeensyTimerTool;
//#define TEENSY_TIMER_TOOL
//#endif

#include <string.h>
#include "FiberCtrl.h"
#include <SPI.h>
#include <Pins.h>


//PeriodicTimer t2;

FiberCtrl::FiberCtrl()
{
  laserState = 30;
}

//synrad does power via an analog pwm, 
// the fiber has 8 input pins to be set for a power value
// as well as aditional pulse frequency settings
// and a different way to enable
uint16_t LastPowerRequest = 0;
uint16_t LastPRFRequest= 0;

void FiberCtrl::update(uint16_t Power)
{
  laserPower = Power;
  handleLaser();
}
void FiberCtrl::update()
{
  handleLaser();
}

void FiberCtrl::begin( int SHReg_Latch_PIN_a,int Laser_Enable_b) 
{
  if(_isHalted)
  {
    if(laserState != 30)
    {
      // ERROR:
      Serial.write("\nLaser begin error: Laser in unexpected state\n");
    }
    else
    {
      //one day ill not use hard coded pins here, one day, not today
      Laser_SHReg_Latch_PIN = SHIFT_REGISTER_LATCH_PIN;
      Laser_Latch_PIN = POWER_LATCH_LASER_PIN;
    
      Laser_Enable_PIN = Laser_Enable_b;
      MO_enable_PIN =LASER_OSCILLATOR_ENABLE_PIN ;
      Amp_enable_PIN = LASER_AMPLIFIER_ENABLE_PIN;
      Laser_Guide_PIN = PILOT_LASER_PIN;
    // pin for setting pulse frequency
      PRF_PIN = LASER_PRF_PIN;

      alarm_0_PIN = LASER_STATUS_PIN1;
      alarm_1_PIN = LASER_STATUS_PIN2;


      pinMode(Laser_SHReg_Latch_PIN, OUTPUT);
      pinMode(Laser_Latch_PIN, OUTPUT);
      SPI.begin(); //look that im using the correct pins on the pcb //implicidly initialized the rest of the pins

      pinMode(Laser_Enable_PIN, OUTPUT);
      pinMode(MO_enable_PIN, OUTPUT);
      pinMode(Amp_enable_PIN, OUTPUT);
      pinMode(Laser_Guide_PIN, OUTPUT);
      pinMode(PRF_PIN, OUTPUT);

      pinMode(alarm_0_PIN, INPUT);
      pinMode(alarm_1_PIN, INPUT);

      
      setPower(0);
      digitalWrite(Laser_Latch_PIN,0);
      setPRF(20000);
      analogWrite(PRF_PIN, 128);
      digitalWrite(MO_enable_PIN,0);
      digitalWrite(Amp_enable_PIN,0);
      digitalWrite(Laser_Guide_PIN,0);

      digitalWrite(Laser_Enable_PIN,1);
      
      // waist of recources - call handle laser at the beginning of each command instead. //t2.begin(this->handleLaser, 50); //50us = 20kHz
    
      laserState = 0;
      laserPower = 0;
      laserPRF = 20000;
      oldlaserPower = 0;
      oldlaserPRF = 20000;
      _isHalted = false;
    }

  }
}

void FiberCtrl::stop()
{
  laserPower = 0;
  digitalWrite(Amp_enable_PIN,0);
  digitalWrite(MO_enable_PIN,0);
  digitalWrite(Laser_Guide_PIN,0);
  digitalWrite(Laser_Enable_PIN,0);
  
  laserState = 30;
  _isHalted = true;
}

bool FiberCtrl::isHalted()
{
  return _isHalted;
}

bool FiberCtrl::isInitiallized()
{
  if(laserState==3 || laserState==4)
    return true;
   else
    return false;
}

void FiberCtrl::handleLaser()
{
  if(isInitiallized())
  {
    #ifdef PIN13_LED_INDICATES_LASER_READY
    digitalWrite(13,1);
    #endif
    if( (laserPower==oldlaserPower) && (laserPRF==oldlaserPRF)) //Nothing changed
      return;
    if(laserPower!=oldlaserPower)
      oldlaserPower = laserPower;
    if(laserPRF!=oldlaserPRF)
      {
        oldlaserPRF = laserPRF;
        setPRF(laserPRF);
      }
  }
  #ifdef PIN13_LED_INDICATES_LASER_READY
  else digitalWrite(13,0);
  #endif
/*   |
 *
 * synrad:
 *  LaserStates:
 *       0 = BEGINWARMUP      - LaserEnable_Pin is HIGH, start the timer and go to warmup
 *       1 = WARMUP           - LaserEnable_Pin is HIGH, state is held for 5 seconds
 *       2 = INIT READY       - When PWM commands Lazing to be less than tickle*2 PWM reconfigures to 1us pulse every 200us (5kHz).
 *       3 = READY            - When PWM commands Lazing to be less than tickle*2 PWM reconfigures to 1us pulse every 200us (5kHz).
 *       4 = LAZING           - When PWM commands Lazing to be more than tickle*2 PWM reconfigures to 20kHz and selected pulse width.
 *       30 = DISABLED        - LaserEnable_Pin is HIGH, state is held for 5 seconds
 * 
 *  fiber:
 *  LaserStates:
 *       0 = BEGINWARMUP      - LaserEnable_Pin is HIGH, go to warmup
 *       1 = WARMUP           - LaserEnable_Pin is HIGH, wait until alarm 1 goes high, set MO_high, wait T_on
 *       2 = INIT READY       - Master Oscillator is running, and waitin on the AMP_enable (GATE) to be set
 *       3 = READY            - Master Oscillator is running, and waitin on the AMA_enable (GATE) to be set
 *       4 = LAZING           - Amp is enabled, Laser runs at selected PRF & Power
 *       30 = DISABLED        - Laser is turned of with its enable pin Laser_enable_PIN != LaserEnable_Pin 
 * 
 * 
 * 
 *  
*/

  int laserEnablePinState = HIGH; //TODO: Impl... digitalRead(Synrad48Ctrl::laserEnable_Pin);
  if(laserEnablePinState == LOW)
  {
    laserState = 30;  //TODO: Fix how to get out of this
    //tickleStart = 0x0;
    //analogWrite(Synrad48Ctrl::laserPWM_OUT_Pin, 0);
  }
  else
  { 
    uint32_t NOW = millis();

    switch (laserState)
    {
      case 0:
      {
        //IN BEGIN WARMUP STATE
        digitalWrite(Laser_Enable_PIN,1); //should be already set, cant hurt
        //fallthrough intended
      }
      case 1:
      {
        // IN WARMUP STATE
        bool ready = ((digitalRead(alarm_1_PIN) == 1)&&(digitalRead(alarm_0_PIN) == 0));
        if(ready)
        {
          digitalWrite(MO_enable_PIN,1);
          laserState = 2;
        }
        else{
          laserState = 1;
        }
        break;
      }
      case 2:
      {
        // IN INIT READY STAT
        laserState = 3;
        //fallthrough intended
      }
      case 3:
      {
        // IN READY STATE
        if(laserPower>6) //hardcoded min value
          laserState = 4;
        else
          break;
        //fallthrough intended
      }
      case 4:
      {
        // IN LAZING STATE
        if(laserPower<=6) //hardcoded min value
        {
          //laserState = 2;
          digitalWrite(Amp_enable_PIN,0);
          setPower(0);
          laserState = 3;
        }
        else
          //if(!(Synrad48Ctrl::laser_Shutter))
          //TODO: FIX IMPLEMENTATION - Sync with LASER_ENABLED and add shutter to LaserController interface
          digitalWrite(Amp_enable_PIN,1);
          setPower(laserPower);
          //else
            //this->set5kPWM();
        break;
      }
      default:
      {
        //laserState = 30;
        //Laser has been disabled
        analogWrite(FiberCtrl::Laser_Enable_PIN, 0);
        break;
      }
    }
    
  }
}

