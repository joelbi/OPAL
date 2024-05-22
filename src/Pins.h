/*
  Pins.h - driver code to handle Pin definitions on PJRC Teensy 4.x board

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

#ifndef PINS_H
  #define PINS_H
  
  //#define LASER_SSR_OUT_PIN not planing to use this, enable works similarly
   
  // pin 0 implicidly used with rx1
  // pin 1 implicidly used with rx1
  #define LASER_STATUS_PIN1 2
  #define LASER_STATUS_PIN2 3
  #define PILOT_LASER_PIN 4
  #define POWER_LATCH_LASER_PIN 5
  #define LASER_PRF_PIN 6 
  #define LASER_AMPLIFIER_ENABLE_PIN 7
  #define LASER_OSCILLATOR_ENABLE_PIN 8
  #define LASER_ENABLE_PIN 9
  #define SHIFT_REGISTER_LATCH_PIN 32
  // pin 10 implicidly used with spi1 but not wired
  // pin 11 implicidly used with spi1
  // pin 12 implicidly used with spi1, but not wired
  // pin 13 implicidly used with spi1

  #define GALVO_SSR_OUT_PIN 15
  // 16 unused
  // sync pin 17 defined in xy2_100.cpp
  // 18 unused
  // x pin 19 defined in xy2_100.cpp
  // 20 unused
  // y pin 21 defined in xy2_100.cpp
  // clock pin 22 defined in xy2_100.cpp
  // 23 unused
  #define LASER_PWM_OUT_PIN 
#endif
