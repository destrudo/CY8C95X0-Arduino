/* Software License:

  CY8C95X0 library definitions and structs
  Copyright (C) 2013  destrudo

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef CY8C95X0_BASE_H
  #define CY8C95X0_BASE_H

  #if ARDUINO >= 100
    #include "Arduino.h"
  #else
    #include "WProgram.h"
  #endif


  #define CY8C95X0_ADDR B0100000
  #define CY8C95X0_EEPROM_ADDR B1010000

  #define CY8C95X0_PWM_0 0
  
  /* Device register map */
  #define REG_PORT_SEL 0x18 //Port select
  #define REG_INT_MASK 0x19 //Interrupt Mask
  #define REG_SEL_PWM_PORT_OUT 0x1A //PWM select for port output
  #define REG_INVERSION 0x1B //Inversion
  #define REG_PIN_DIR 0x1C //Pin direction input/output
  #define REG_DM_PU 0x1D //Drive mode pull up
  #define REG_DM_PD 0x1E //Drive mode pull down
  #define REG_DM_ODH 0x1F //Drive mode open drain high
  #define REG_DM_ODL 0x20 //Drive mode open drain low
  #define REG_DM_STRONG 0x21 //Drive mode strong
  #define REG_DM_SLOW 0x22 //Drive mode slow strong
  #define REG_DM_HIZ 0x23 //Drive mode High-Z
  #define REG_SEL_PWM 0x28 //PWM Select
  #define REG_CONF_PWM 0x29 //PWM config
  #define REG_PERI_PWM 0x2A //PWM Period
  #define REG_PW_PWM 0x2B //Pulse width PWM
  #define REG_PROG_DIV 0x2C //Programmable Divider
  #define REG_ENABLE 0x2D //Enable (EERO, EEE, WDE)
  #define REG_DEV_STATUS 0x2E //Device ID/Status
  #define REG_WATCHDOG 0x2F //Watchdog
  #define REG_CMD 0x30 //Command
  
  /* Interrupt Status */
  #define REG_INT_STAT_0 0x10
  #define REG_INT_STAT_1 0x11
  #define REG_INT_STAT_2 0x12
  #define REG_INT_STAT_3 0x13
  #define REG_INT_STAT_4 0x14
  #define REG_INT_STAT_5 0x15
  #define REG_INT_STAT_6 0x16
  #define REG_INT_STAT_7 0x17
  
  /* Command registers */
  #define REG_CMD 0x30
  #define REG_CMD_STORE 0x01
  #define REG_CMD_RESTORE 0x02
  #define REG_CMD_STORE_D 0x03 //In the datasheet, pg 14, probably not useful on the large scale
  #define REG_CMD_READ_D 0x04 //This is good for dumping the chip's config
  #define REG_CMD_WRITE 0x05
  #define REG_CMD_READ 0x06
  #define REG_CMD_RECONF 0x07
  
  
  /* Pin group register mapping */
  /* Input */
  #define REG_GI0 0x00
  #define REG_GI1 0x01
  #define REG_GI2 0x02
  #define REG_GI3 0x03
  #define REG_GI4 0x04
  #define REG_GI5 0x05
  #define REG_GI6 0x06
  #define REG_GI7 0x07
  /* Output */
  #define REG_GO0 0x08
  #define REG_GO1 0x09
  #define REG_GO2 0x10
  #define REG_GO3 0x11
  #define REG_GO4 0x12
  #define REG_GO5 0x13
  #define REG_GO6 0x14
  #define REG_GO7 0x15
  
  /* pin group mapping */
  #define PIN_G0 0x00
  #define PIN_G1 0x01
  #define PIN_G2 0x02
  #define PIN_G3 0x03
  #define PIN_G4 0x04
  #define PIN_G5 0x05
  #define PIN_G6 0x06
  #define PIN_G7 0x07
  
  /* pin mapping */
  #define PIN_0 0x00
  #define PIN_1 0x01
  #define PIN_2 0x02
  #define PIN_3 0x03
  #define PIN_4 0x04
  #define PIN_5 0x05
  #define PIN_6 0x06
  #define PIN_7 0x07
  
  /* Misc defs */
  #define ALL_PINS 0xFF
  #define NO_PINS 0x00
  
  
  struct pin_t {
    byte group;
    byte pin;
  };
  
  //PWM config structure
  struct pwm_t {
    byte clock;
    byte period;
    byte pw;
  };
  
  //pin drive type struct
  struct drive_t {
    byte pullup;
    byte pulldown;
    byte odhigh;
    byte odlow;
    byte strong;
    byte slow;
    byte hiz;
  };

#endif
