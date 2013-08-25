/* Software License:

  Arduino CY8C95X0 library header
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


#ifndef CY8C95X0_H
  #define CY895X0_H

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "CY8C95X0_BASE.h"
#include <stdarg.h>

class CY8C95X0
{
 private:
  uint8_t pin_c; //Number of pins per group
  uint8_t group_c; //Number of groups
  uint8_t pwm_c; //number of pwm controllers
  
  /* If you want a slightly smaller memory footprint, enable these and where I marked in begin() or the constructor, depending on how
   * You initialized the class.  Dynamic memory is good, when your chip supports it. */
  // byte * pinstates; //State of all pins on a group-by-group basis
  // byte * pwmstates; //PWM'd state of each above pin
  // drive_t * drivestates;
  // byte * intstates; //State of pin interrupts
  // byte * invstates; //State of pin inversions
  // byte * pindirections; //State of pin directions
  // pwm_t * pwmconf; //PWM configuration (per-channel)
  
  /* IF you wanted that smaller mem footprint discussed above, you'll need to comment all the arrays below */
  byte pinstates[8]; //State of all pins on a group-by-group basis
  byte pwmstates[8]; //PWM'd state of each above pin
  drive_t drivestates[8];
  byte intstates[8]; //State of pin interrupts
  byte invstates[8]; //State of pin inversions
  byte pindirections[8]; //State of pin directions
  pwm_t pwmconf[16]; //PWM configuration (per-channel)
  byte divider;
  //byte enable; //Unused, chip uses it to restrict writes to the chip eeprom among other things
  //byte CRC; //Unused CRC, maybe later
  //byte groupmodes; //Mode of each group, 0 is output, 1 is input, on a bit-by-bit basis  //Unused group modeset, will likely be pulled.
  byte address; //Device address
  
 public:
  CY8C95X0(uint8_t, uint8_t = 0);
  CY8C95X0();
  void begin(uint8_t, uint8_t = 0);
  
  pin_t pinTranslate(uint8_t);
  void resetChip();
  void saveChip();
  boolean validPort(byte);
  void resetRegister();
  void rawWrite(int, ...);
  void __getConfig();
  void __portSelect(byte);
  byte __getDivider();
  byte __getOutput(uint8_t);
  byte __getInput(uint8_t);

  /* these are untested */
  /***********************
   * Interrupt functions *
   ***********************/
  byte __interrupt(uint8_t);
  byte _interrupt(uint8_t);
  boolean interrupt(pin_t);
  boolean interrupt(uint8_t); 
  boolean interrupt(uint8_t, uint8_t);
  void __interruptMask(uint8_t);
  void _interruptMask(uint8_t, byte);
  void interruptMask(pin_t, boolean);
  void interruptMask(uint8_t, uint8_t, boolean);
  void interruptMask(uint8_t, boolean);
  byte __getInterruptMask(uint8_t);
  byte _getInterruptMask(uint8_t);
  boolean getInterruptMask(pin_t);
  boolean getInterruptMask(uint8_t, uint8_t);
  boolean getInterruptMask(uint8_t);

  /*****************************
   * Input Inversion Functions *
   *****************************/
  byte __getInvStates(uint8_t);
  byte _getInversionGroup(uint8_t);
  boolean getInversion(pin_t);
  boolean getInversion(uint8_t);
  boolean getInversion(uint8_t, uint8_t);
  void __invert(uint8_t);
  void _invert(uint8_t, uint8_t);
  void invertGroup(uint8_t,byte=0x01);
  void invert(pin_t, byte=0x01);
  void invert(uint8_t, uint8_t);
  void invert(uint8_t);
  void invertT(uint8_t);
  void invertT(uint8_t, uint8_t);
  void invertOff(uint8_t, uint8_t);
  void invertOff(uint8_t);

  /*****************
   * PWM functions *
   *****************/
  int pinPWM(pin_t);
  /* Low level, consider making private */
  pwm_t __getPWMConfig(uint8_t);
  byte __getPortPWM(uint8_t);
  void __pwmSelect(byte);
  void __pwmConfigSelect(byte);
  void __pwmClockSel(byte);
  void __pwmConfigPeriod(byte);
  void __pwmConfigPulseWidth(byte);
  void __pwmConfigDivider(byte);
  void _pwmConfig(byte, byte, byte = 0xFF);
  void _pwmConfig(byte, byte, byte, byte);
  void _pwmSelect(pin_t, boolean);
  void pwmConfig(byte, byte, byte, byte);
  void pwmConfig(uint8_t, uint8_t);
  void pwmSelect(uint8_t, boolean);
  void pwmSelect(uint8_t, uint8_t, boolean);
  void pwmSelect(boolean);
  void analogWrite(pin_t, uint8_t);
  void analogWrite(byte, byte, uint8_t);
  void analogWrite(uint8_t, uint8_t);
  
  /************************
   * Drive mode functions *
   ************************/
  drive_t __getDrive(uint8_t);
  void __driveSelect(byte, byte); //pins byte, mode
  void _driveSelectPin(pin_t, byte); //pin, mode
  void driveSelectAll(byte); //mode
  void driveSelectGroup(uint8_t, byte); //group, mode
  void driveMode(uint8_t, byte);

  /***************************
   * Pin direction functions *
   ***************************/
  byte __getPortDirection(uint8_t);
  void __pinDirection(byte);
  void _pinMode(pin_t, boolean);
  void pinMode(uint8_t, boolean);
  void pinMode(uint8_t, uint8_t, boolean);
  
  /*****************
   * I/O Functions *
   *****************/
  void __digitalH(byte, byte);
  boolean _digitalRead(pin_t);  
  boolean digitalRead(uint8_t);
  boolean digitalRead(uint8_t, uint8_t);
  
  void _digitalWrite(pin_t, boolean);
  void digitalWrite(uint8_t, boolean);
  void digitalWrite(uint8_t, uint8_t, boolean);
  void digitalWrite(boolean);

};

#endif
