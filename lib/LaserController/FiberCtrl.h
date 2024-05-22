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

#pragma once

#ifdef __AVR__
#error "Sorry, this only works on 32 bit Teensy boards.  AVR isn't supported."
#endif

#if TEENSYDUINO < 121
#error "Minimum PJRC Teensyduino version 1.21 is required"
#endif

#ifndef FIBERCTRL_h
#define FIBERCTRL_h

#include <Arduino.h>
#include "LaserController.h"
//teensy SPI library might be called different
#include <SPI.h>

class FiberCtrl : public LaserController {
	public:
		FiberCtrl();
    void begin(int PWM_OUT_Pin, int PSU_SSR_Pin);
    void stop(void);
    bool isInitiallized();
    bool isHalted();
		void handleLaser();
		void update(uint16_t Power);
		void update();
   
	private:
    int                     laserState;

    // pins for setting power
    int Laser_ShReg_Data_PIN;
    int Laser_ShReg_Clock_PIN;
    int Laser_SHReg_Latch_PIN;
    int Laser_Latch_PIN;
    // pins for enabling
    int Laser_Enable_PIN;
    int MO_enable_PIN;
    int Amp_enable_PIN;
    int Laser_Guide_PIN;
    // pin for setting pulse frequency
    int PRF_PIN;

    int alarm_0_PIN;
    int alarm_1_PIN;

    
    int                     laserPower          = 0;
    int                     laserPRF            = 20000;
    int                     oldlaserPower       = 0;
    int                     oldlaserPRF         = 0;
    bool                    _isHalted           = true;

/*
@param PWM 
void set20kPWM(int PWM) {
  if(currentFreq != 20000) {
    currentFreq = 20000;
    analogWriteFrequency(laserPWM_OUT_Pin, 20000);
  }
  analogWrite(laserPWM_OUT_Pin, laserPWM); //Output Laser
}


void set5kPWM() {
  if(currentFreq != 5000) {
    currentFreq = 5000;
    analogWriteFrequency(laserPWM_OUT_Pin, 5000);
  }
  analogWrite(laserPWM_OUT_Pin, Synrad48Ctrl::ticklePWM); //Output Trickle
}
*/

void setPower(int Power){
  if((0 < Power)||(Power < 255)){
    //invalid power? do i print an error here?
  }else{
    //include a correct spi library
    byte data = (byte)Power;
    SPISettings settings(1000000, MSBFIRST, SPI_MODE0);
    SPI.beginTransaction(settings);
    digitalWrite(Laser_SHReg_Latch_PIN, LOW);
    digitalWrite(Laser_Latch_PIN, LOW);
    SPI.transfer(data); //hope does type casting works
    digitalWrite(Laser_SHReg_Latch_PIN, HIGH);
    //do i need a small delay here?
    digitalWrite(Laser_Latch_PIN, HIGH);
    SPI.endTransaction();
  }
}

void setPRF(int PRF){
  if((20000 < PRF)||(PRF < 100000)){
    //invalid prf? do i print an error here?
  }else{
    analogWriteFrequency(PRF_PIN, PRF);
  }
}

};
#endif
