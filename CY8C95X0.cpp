/* Software License:

  Arduino CY8C95X0 library method file
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


#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <Wire.h>
#include <stdarg.h>
#include "CY8C95X0.h"
#include "CY8C95X0_BASE.h"

/* CY8C95X0()
 * Default constructor method which does essentially nothing
 * -The wire.begin command, as far as 1.01/3/5 are concerned, seems to be
 * meaningless.  It's there because most libs call it, and if it's required
 * for compat, I'd rather have it than not.
 */
CY8C95X0::CY8C95X0()
{
  Wire.begin();
}

/* CY8C95X0(uint8_t chip, uint8_t A)
 * This constructor actually calls begin() within it.
 * -Use this if you're allocating it with the 'new' operator; but make sure 
 * that it gets called AFTER Wire.begin() and your other libraries have 
 * .begin()'d.  If not, Strange things will happen.
 */
CY8C95X0::CY8C95X0(uint8_t chip, uint8_t A)
{
  CY8C95X0::begin(chip, A);
}

/* begin(uint8_t chip, uint8_t A)
 * This method fills up our local variables with data
 * -Address is set here, the A parameter which is the optional address is set.
 *   *If your optional address is greater than 31 (If you're using the A5 pin)
 *   you MUST state the COMPLETE 7 bit address in A
 */
void CY8C95X0::begin(uint8_t chip, uint8_t A)
{
  address = CY8C95X0_ADDR;
  
  /* Address handling */
  if(A)
  {
    if( A <= 31 ) address |= A;
    else address = A;
  }

  /* Feel free to comment the next line out, I just prefer having it on */
  resetChip(); //Clear the chip's current config, roll back to defaults.

  switch(chip)
  {
    case 20:
      pin_c = 20;
      pwm_c = 4;
      group_c = 3;
      break;
    case 40:
      pin_c = 40;
      pwm_c = 8;
      group_c = 5;
      break;
    case 60:
      pin_c = 60;
      pwm_c = 16;
      group_c = 8;
      break;
  }
  
  /* IF you wanted to use dynamic allocation for the pin states, uncomment these out. */
  // pinstates = new byte[group_c]; //Allocate the state arrays
  // pwmstates = new byte[group_c]; 
  // pwmconf = new pwm_t[pwm_c];
  // drivestates = new drive_t[group_c];
  // intstates = new byte[group_c];
  // invstates = new byte[group_c];

  __getConfig(); //Update the current config
}

/* pinTranslate(uint8_t pin)
 * This method translates a number to pin and returns a pin_t
 * -If the pin is not found, it'll return a pin_t with group and pin set
 * to 255
 */
pin_t CY8C95X0::pinTranslate(uint8_t pinIn)
{
  pin_t pin = {255,255};
  if((pinIn + 1) > pin_c) return pin; //If pin is outside values, exit.
  if(pinIn > 19) pinIn += 4; //Since port 2 only has a nibble, we need to jump ahead after that point
  pin.pin = byte(pinIn % 8); //set pin
  pin.group = byte(pinIn / 8); //set pin group
  return pin;
}

/* resetChip()
 * This method resets the chip to eeprom-stored defaults
 */
void CY8C95X0::resetChip()
{
  rawWrite(2, REG_CMD, REG_CMD_RESTORE);
  rawWrite(2, REG_CMD, REG_CMD_RECONF);
}

/* saveChip()
 * This method saves the current chip state to the eeprom
 */
void CY8C95X0::saveChip()
{
  //It's obvious at this point that this function does nothing at this moment
  delay(1000);
}

/* validPort(byte port)
 * This method returns a true/false value depending on whether or not 'port' is
 * within the bounds of the chip
 */
boolean CY8C95X0::validPort(byte port)
{
   if( (port + 1) > group_c) return false;
   return true;
}
/* resetRegister()
 * This method writes a 0 to the chip so that the next read or write will start
 * at address 0x00
 */
void CY8C95X0::resetRegister()
{
  Wire.beginTransmission(address);
  Wire.write((uint8_t)0);
  Wire.endTransmission();
}

/* rawWrite(int cmds, ...)
 * This method writes any number of values to the device.
 * -arguments:
 *  *cmds is number of commands to be sent
 *  *varable arguments will be written in order
 * -This is pretty low level, but if you need to do something I didn't
 * provide, this is how.
 */
void CY8C95X0::rawWrite(int cmds, ...)
{
  resetRegister();
  va_list argList;
  va_start(argList, cmds);
  byte call = 0;
  Wire.beginTransmission(address);
  for(;cmds;cmds--)
  {
    call = (byte)va_arg(argList,int);
    Wire.write((uint8_t)call);
  }
  Wire.endTransmission();
  va_end(argList);
}

/* __getConfig()
 * This method populates most of the local variables with that stored on the chip
 */
void CY8C95X0::__getConfig()
{
  byte tmp = __getPortDirection(1);
  for(int i = 0; i < group_c; i++)
  {
    pindirections[i] = __getPortDirection(i);
    intstates[i] = __getInterruptMask(i);
    pwmstates[i] = __getPortPWM(i);
    drivestates[i] = __getDrive(i);
    invstates[i] = __getInvStates(i);
  }
  for(int i = 0; i < pwm_c; i++)
  {
    pwmconf[i] = __getPWMConfig(i);
  }
}

/* _portSelect(byte port)
 * This method writes a command to the chip which sets port in the port select
 * register
 */
void CY8C95X0::__portSelect(byte port)
{
  if(!validPort(port)) return; //Return if port out of range
  rawWrite(2,REG_PORT_SEL,port);
}

/* __getDivider()
 * This method returns the divider value which is stored on the chip
 */
byte CY8C95X0::__getDivider()
{
  byte tmp;
  rawWrite(1, REG_PROG_DIV);
  Wire.requestFrom(address, uint8_t(1));
  if(Wire.available()) tmp = Wire.read();
  return tmp;
}

/* __getOutput(uint8_t group)
 * This method gets the current state of group's output register
 */
byte CY8C95X0::__getOutput(uint8_t group)
{
  byte tmp;
  rawWrite(1, REG_GO0 + group);
  Wire.requestFrom(address, uint8_t(1));
  if(Wire.available()) tmp = Wire.read();
  return tmp;
}

/* __getInput(uint8_t group)
 * This method gets the current state of the group's input register
 */
byte CY8C95X0::__getInput(uint8_t group)
{
  byte tmp;
  rawWrite(1, (0x00 + group));
  Wire.requestFrom(address, uint8_t(1));
  if(Wire.available()) tmp = Wire.read();
  return tmp;
}


  /* these are untested */
  /***********************
   * Interrupt functions *
   ***********************/

/* __interrupt(uint8_t group)
 * This method returns the current interrupt status of a port's pins
 */
byte CY8C95X0::__interrupt(uint8_t group)
{
  __portSelect(group);
  rawWrite(1, REG_INT_STAT_0 + group);
  Wire.requestFrom(address, uint8_t(1));
  if(Wire.available()) return Wire.read();
  return 0;
}
/* _interrupt(uint8_t group)
 * This method returns the current interrupt status of a port's pins
 * -This function verifies that the group given is sane, and then calls
 * the __interrupt function
 */
byte CY8C95X0::_interrupt(uint8_t group)
{
  if(group >= group_c) return 0x00;
  return __interrupt(group);
}
/* interrupt(pin_t pin)
 * This method returns a boolean state which is the interrupt status of
 * the parameter pin.
 */
boolean CY8C95X0::interrupt(pin_t pin)
{
  boolean tmp = false;
  tmp = __interrupt(pin.group) & (1 << pin.pin);
  return tmp;
}
/* interrupt(uint8_t group, uint8_t pin)
 * This method returns a boolean state which is the interrupt status of
 * the parameter pin in group group.
 */
boolean CY8C95X0::interrupt(uint8_t group, uint8_t pin)
{
  pin_t tmp = {group, pin};
  return (interrupt(tmp));
}
/* interrupt(uint8_t pin)
 * This method returns a boolean state which is the interrupt status of
 * the parameter pin
 */
boolean CY8C95X0::interrupt(uint8_t pin)
{
  return interrupt(pinTranslate(pin));
}
/* __getInterruptMask(uint8_t group)
 * Returns a byte of the current masks set on group
 */
byte CY8C95X0::__getInterruptMask(uint8_t group)
{
  __portSelect(group);
  rawWrite(1, REG_INT_MASK);
  Wire.requestFrom(address, uint8_t(1));
  if(Wire.available()) return Wire.read();
  return 0x00;
}
/* _getInterruptMask(uint8_t group)
 * Returns a byte of the current masks set on a group
 * -This function performs a sanity check on the group
 */
byte CY8C95X0::_getInterruptMask(uint8_t group)
{
  if(group >= group_c) return 0x00;
  return __getInterruptMask(group);
}
/* getInterruptMask(pin_t pin)
 * This method returns a boolean state which is that of the pin's
 * current interrupt mask
 */
boolean CY8C95X0::getInterruptMask(pin_t pin)
{
  byte tmp = __getInterruptMask(pin.group);
  return (tmp & (1 << pin.pin));
//return (__getInterruptMask(pin.group) & (1 << pin.pin));
}
/* getInterruptMask(uint8_t group, uint8_t pin)
 * This method returns a boolean state which is that of the pin's
 * current interrupt mask
 */
boolean CY8C95X0::getInterruptMask(uint8_t group, uint8_t pin)
{
  pin_t tmp = {group, pin};
  return getInterruptMask(tmp);
}
/* getInterruptMask(uint8_t pin)
 * This method returns a boolean state which is that of the pin's
 * current interrupt mask
 */
boolean CY8C95X0::getInterruptMask(uint8_t pin)
{
  return getInterruptMask(pinTranslate(pin));
}


  /*****************************
   * Input Inversion Functions *
   *****************************/

/* __getInvStates(uint8_t group)
 * This method returns a byte which is the current input inversion
 * state of the port.
 */
byte CY8C95X0::__getInvStates(uint8_t group)
{
  byte tmp;
  __portSelect(group);
  rawWrite(1, REG_INVERSION);
  Wire.requestFrom(address, uint8_t(1));
  if(Wire.available()) tmp = Wire.read();
  return tmp;
}
/* Everybody in this world is just like me. */
/* _getInversionGroup(uint8_t group)
 * This method returns the inversion state of the group
 * -Performs a sanity check on the input group
 */
byte CY8C95X0::_getInversionGroup(uint8_t group)
{
  if(group >= group_c) return 0x00;
  return __getInvStates(group);
}
/* getInversion(pin_t pin)
 * This method returns the inversion state of a particular pin
 */
boolean CY8C95X0::getInversion(pin_t pin)
{
  boolean tmp = false;
  tmp = __getInvStates(pin.group) & (1 << pin.pin);
  return tmp;
}
/* getInversion(uint8_t group, uint8_t pin)
 * This method returns the inversion state of a particular pin
 */
boolean CY8C95X0::getInversion(uint8_t group, uint8_t pin)
{
  pin_t tmp = {group,pin};
  return getInversion(tmp);
}
/* getInversion(uint8_t pin)
 * This method returns the inversion state of a particular pin
 */
boolean CY8C95X0::getInversion(uint8_t pin)
{
  return getInversion(pinTranslate(pin));
}
/* __invert(uint8_t group)
 * This method is a raw value write to the device to invert
 * the pins of a selected group
 */
void CY8C95X0::__invert(uint8_t group)
{
  rawWrite(2,REG_INVERSION,group);
}
/* _invert(uint8_t group, uint8_t pins)
 * This method sets inversion pins on group
 * -Performs sanity check on group parameter
 */
void CY8C95X0::_invert(uint8_t group, uint8_t pins)
{
  if(group >= group_c) return;
  __portSelect(group);
  __invert(pins);
}
/* invertGroup(uint8_t group, byte mode)
 * This method sets inversion pins on a group with a particular mode
 * -if mode is 0x00, it shuts off the inversion
 * -if mode is 0x01, it turns off inversion on all ports
 * -if mode is 0x02 or more, the current state of each pin's inversion
 * is inverted.
 */
void CY8C95X0::invertGroup(uint8_t group, byte mode)
{
  if(mode == 0x00) _invert(group,0x00);
  else if(mode == 0x01) _invert(group,0xFF);
  else
  {
    if(group >= group_c) return; //We can't have someone addressing out of bounds
    invstates[group] ^= 0xFF;
    _invert(group,invstates[group]);
  }
}
/* invert(pin_t pin, byte mode)
 * This method sets inversion on a single pin
 * -if mode is 0x00, the inversion is shut off on that pin
 * -if mode is 0x01, the inversion is turned on on the pin
 * -if mode ix 0x02 or more, the pin's inversion state is inverted.
 */
void CY8C95X0::invert(pin_t pin, byte mode)
{
  if(pin.group >= group_c) return;
  if(mode == 0x00)
  {
    invstates[pin.group] &= ~(1 << pin.pin);
  }
  else if (mode == 0x01)
  {
    invstates[pin.group] |= 1 << pin.pin;
  }
  else
  {
    invstates[pin.group] ^= 1 << pin.pin;
  }
  _invert(pin.group,invstates[pin.group]);
}
/* invert(uint8_t group, uint8_t pin)
 * This method sets inversion on, for a single pin
 */
void CY8C95X0::invert(uint8_t group, uint8_t pin)
{
  pin_t tmp = {group,pin};
  invert(tmp);
}
/* invert(uint8_t pin)
 * This method sets inversion on, for a single pin
 */
void CY8C95X0::invert(uint8_t pin)
{
  invert(pinTranslate(pin));
}
/* invertT(uint8_t pin)
 * This method toggles inversion for a single pin
 */
void CY8C95X0::invertT(uint8_t pin)
{
  invert(pinTranslate(pin),0x02);
}
/* invertT(uint8_t group, uint8_t pin)
 * This method toggles inversion for a single pin
 */
void CY8C95X0::invertT(uint8_t group, uint8_t pin)
{
  pin_t tmp = {group,pin};
  invert(tmp,0x02);
}
/* invertOff(uint8_t pin)
 * This method turns off inversion for a single pin
 */
void CY8C95X0::invertOff(uint8_t pin)
{
  invert(pinTranslate(pin),0x00);
}
/* invertOff(uint8_t group, uint8_t pin)
 * This method turns off inversion for a single pin
 */
void CY8C95X0::invertOff(uint8_t group, uint8_t pin)
{
  pin_t tmp = {group,pin};
  invert(tmp,0x00);
}


  /*****************
   * PWM functions *
   *****************/


/* pinPWM(pin_t pin)
 * This method accepts a pin type and returns the pwm controller for said pin.
 * -It does run a sanity check on the pin.group and pin.pin values
 * -At the moment, it only supports a matrix for the 60 pin model, but others
 *  will be added at a later date.
 */
int CY8C95X0::pinPWM(pin_t pin)
{
  if( (pin.group >= group_c) || (pin.pin >= MAX_PIN) ) return;

  /* 60 pin pwm matrix
       P0   P1   P2   P3   P4   P5   P6   P7
  G0   7    5    3    1    7    5    3    1
  G1   6    4    2    0    6    4    2    0    
  G2   14   12   8    11
  G3   7    5    3    1    15   13   11   9
  G4   6    4    2    0    14   12   10   8
  G5   10   8    11   9    12   14   13   15
  G6   0    1    2    3    4    5    6    7
  G7   8    9    10   11   12   13   14   15
  */
  byte level[8][8] = { {7,5,4,1,7,5,3,1},
                       {6,4,2,0,6,4,2,0},
                       {14,12,8,11,255,255,255,255}, 
                       {7,5,3,1,15,13,11,9},
                       {6,4,2,0,14,12,10,8},
                       {10,8,11,9,12,14,13,15},
                       {0,1,2,3,4,5,6,7},
                       {8,9,10,11,12,13,14,15} };
  return level[pin.group][pin.pin];
}
/* __getPWMConfig(uint8_t circuit)
 * This method returns a pwm_t struct containing circuit pwm's settings
 * -There is no software sanity check, however the hardware will likely not
 * return sensible data if a circuit is used greater than the pwm_c variable
 */
pwm_t CY8C95X0::__getPWMConfig(uint8_t circuit)
{
  pwm_t tmp;
  rawWrite(2, REG_SEL_PWM, circuit);
  Wire.requestFrom(address, uint8_t(3));
  if(Wire.available()) tmp.clock = Wire.read();
  if(Wire.available()) tmp.period = Wire.read();
  if(Wire.available()) tmp.pw = Wire.read();
  return tmp;
}
/* __getPortPWM(uint8_t group)
 * This method returns a byte containing the currents pins with PWM enabled
 * (The 1's) and the disabled (0's) pins within a given group. 
 * -There are no sanity checks, but the hardware will react poorly to a group
 * getting selected that is greater than group_c
 */
byte CY8C95X0::__getPortPWM(uint8_t group)
{
  byte tmp;
  __portSelect(group);
  rawWrite(1, REG_SEL_PWM_PORT_OUT);
  Wire.requestFrom(address, uint8_t(1));
  if(Wire.available()) tmp = Wire.read();
  return tmp;
}
/* __pwmSelect(byte pins)
 * This method does a simple write of the PWM Select command followed by a pin
 * byte.
 * -There is no sanity check
 * -It does not call the port open, initiating this command alone will do nothing
 * that anyone expects.
 */
void CY8C95X0::__pwmSelect(byte pins)
{
  rawWrite(2,REG_SEL_PWM_PORT_OUT,pins); //Send the command
}
/* __pwmConfigSelect(byte controller)
 * This method is used to initiate a command write to the (controller) pwm
 * controller for purposes of changing settings.
 * -There is no sanity check, running this will result in the hardware waiting
 * for further data (Hanging until another set of bytes comes along)
 */
void CY8C95X0::__pwmConfigSelect(byte controller)
{
  rawWrite(2,REG_SEL_PWM,controller);
}



///////////////////////THIS IS WRONG/////////////////////////////////

/* __pwmClockSel(byte pwmController)
 * This method is for initiating reading, or writing clock data from/to the 
 * pwm (controller) set
 * -There is no sanity check, the hardware will bug out if this is used without
 * proper commands sent.
 */
void CY8C95X0::__pwmClockSel(byte controller)
{
  rawWrite(2,REG_CONF_PWM,controller);
}




/* __pwmConfigPeriod(byte period)
 * This method is used for sending period data to the previously selected
 * controller
 */
void CY8C95X0::__pwmConfigPeriod(byte period)
{
  rawWrite(2,REG_PERI_PWM,period);
}
/* __pwmConfigPulseWidth(byte pulsewidth)
 * This method is used for sending pulse width data to the previously selected
 * controller
 */
void CY8C95X0::__pwmConfigPulseWidth(byte pulsewidth)
{
  rawWrite(2,REG_PW_PWM,pulsewidth);
}
/* __pwmConfigDivider(byte divider)
 * This method is used for writing data to the divider register
 * -Using this without understanding is a bad idea, it can mess with things in
 * a bad way.  Its intent is for using the 00000100b setting on the pwm config
 * to achieve custom frequencies on the controller.  Problem is, it applies to
 * all controllers.  So setting it will mess with the other pwms.
 */
void CY8C95X0::__pwmConfigDivider(byte divider)
{
  rawWrite(2,REG_PROG_DIV,divider);
}
/* _pwmConfig(byte circuit, byte pwm, byte period)
 * This method is used for configuring the pwm controllers
 *
 * -The period is a completely optional argument since so few people will 
 * realistically need a higher frequency than 130hz.  But incase You need
 * some math pointers:
 *   1khz = 0x22 to the register
 *   142hz = 0xF0, 2khz = 0x11, 435hz = 0x4F
 */
void CY8C95X0::_pwmConfig(byte circuit, byte pw, byte period)
{
  pwm_t tmp = pwmconf[circuit]; //Copy the current config

  if(period != 0xFF) //If period is set
  {
    tmp.period = period; //Copy our period
  }
  else //If period was not set, copy the old data
  {
    tmp.period = pwmconf[circuit].period;
  }
  tmp.pw = pw;
  pwmconf[circuit] = tmp; //Save the config
  __pwmConfigSelect(circuit); //Write it to the chip
  __pwmConfigPeriod(pwmconf[circuit].period);
  __pwmConfigSelect(circuit);
  __pwmConfigPulseWidth(pwmconf[circuit].pw);
}
/* _pwmConfig(byte circuit, byte clock, byte period, byte pw)
 * This method is for controlling the pwm circuits, including modification
 * of the clock.
 * -This accepts a clock source, a period, a pulse width, and a circuit.
 * There's no default anything here, they must all be set.
 * Read the data sheet if you intend to use this.
 */
void CY8C95X0::_pwmConfig(byte circuit, byte clock, byte period, byte pw)
{
  pwm_t tmp = {clock, period, pw};
  pwmconf[circuit] = tmp;
  __pwmConfigSelect(circuit);
  __pwmClockSel(clock);
  __pwmConfigSelect(circuit);
  __pwmConfigPeriod(period);
  __pwmConfigSelect(circuit);
  __pwmConfigPulseWidth(pw);
}
/* _pwmSelect(pin_t pin, boolean mode)
 * This method enables or disables the PWM circuits on a pin
 */
void CY8C95X0::_pwmSelect(pin_t pin, boolean mode)
{
  if(mode == HIGH)
  {
    pwmstates[pin.group] |= 1 << pin.pin;  //Set the PWM state local 'register'
    pinstates[pin.group] |= 1 << pin.pin;  //Set the pinstates local 'register' so that if another digitalWrite occurs, our pwm isn't reset.
  }
  else
  {
    pwmstates[pin.group] &= ~(1 << pin.pin);
    pinstates[pin.group] &= ~(1 << pin.pin);
  }
  __portSelect(pin.group);
  __pwmSelect(pwmstates[pin.group]);
}
/* pwmConfig(byte circuit, byte clock, byte period, byte pw)
 * This method handles configuration of a particular pwm circuit
 * -The method sanity checks inputs
 */
void CY8C95X0::pwmConfig(byte circuit, byte clock, byte period, byte pw)
{
  if(circuit >= pwm_c) return;
  if(pw >= period) return;
  _pwmConfig(circuit, clock, period, pw);
}
/* pwmConfig(byte circuit, byte duty)
 * This method modifies the duty cycle (Pulse width) of a pwm circuit
 */
void CY8C95X0::pwmConfig(byte circuit, byte duty)
{
  if(circuit >= pwm_c) return; //If the circuit requested is higher than that on our chip, exit.
  if(duty >= pwmconf[circuit].period) return; //Pulse width, according to the datasheet, should always be less than the period by at least 1
  _pwmConfig(circuit, duty);
}
/* pwmSelect(uint8_t pinIn, boolean mode)
 * This method turns off or on a pin's PWM mode
 */
void CY8C95X0::pwmSelect(uint8_t pinIn, boolean mode)
{
  pin_t pin_v = pinTranslate(pinIn);
  _pwmSelect(pin_v, mode);
}
/* pwmSelect(uint8_t group, uint8_t pinIn, boolean mode)
 * This method turns off or on pwm for a complete group
 */
void CY8C95X0::pwmSelect(uint8_t group, uint8_t pinIn, boolean mode)
{
  pin_t pinOut = {group,pinIn};
  _pwmSelect(pinOut, mode);
}
/* pwmSelect(boolean mode)
 * This method enables or disables pwm for all the pins on the device
 */
void CY8C95X0::pwmSelect(boolean mode)
{
  for(int i = 0; i < group_c; i++)
  {
    __portSelect(i);
    if(mode == HIGH) __pwmSelect(ALL_PINS);
    else __pwmSelect(NO_PINS);
  }
}
/* analogWrite(pin_t pin, uint8_t value)
 * This method writes a duty cycle value to the pwm controller of the specified
 * pin
 * -It is the same as the analogWrite function inside arduino
 */
void CY8C95X0::analogWrite(pin_t pin, uint8_t value)
{
  _pwmConfig(pinPWM(pin),value);
}
/* analogWrite(byte group, byte pin, uint8_t value)
 * This method writes a duty cycle value to the pin in group's pwm controller
 */
void CY8C95X0::analogWrite(byte group, byte pin, uint8_t value)
{
  pin_t tmp = {group, pin};
  _pwmConfig(pinPWM(tmp), value);
}
/* analogWrite(uint8_t pin, uint8_t value)
 * This method writes a duty cycle value to the pin's pwm controller
 */
void CY8C95X0::analogWrite(uint8_t pin, uint8_t value)
{
  _pwmConfig(pinPWM(pinTranslate(pin)),value);
}


  /************************
   * Drive mode functions *
   ************************/


/* __getDrive(uint8_t group)
 * This method gets the drive modes of the specified group
 */
drive_t CY8C95X0::__getDrive(uint8_t group)
{
  drive_t tmp;
  __portSelect(group);
  rawWrite(1, REG_DM_PU);
  Wire.requestFrom(address, uint8_t(7));
  if(Wire.available()) tmp.pullup = Wire.read();
  if(Wire.available()) tmp.pulldown = Wire.read();
  if(Wire.available()) tmp.odhigh = Wire.read();
  if(Wire.available()) tmp.odlow = Wire.read();
  if(Wire.available()) tmp.strong = Wire.read();
  if(Wire.available()) tmp.slow = Wire.read();
  if(Wire.available()) tmp.hiz = Wire.read();
  return tmp;
}
/* __driveSelect(byte pins, byte mode)
 * This method sets drive modes for those active pins
 */
void CY8C95X0::__driveSelect(byte pins, byte mode)
{
  rawWrite(2,mode,pins);
}
/* _driveSelectPin(pin_t pin, byte mode)
 * This method handles a drive select on a single pin
 */
void CY8C95X0::_driveSelectPin(pin_t pin, byte mode)
{
  if( (mode <= REG_DM_PU) && (mode >= REG_DM_HIZ) ) return; //If the mode handed to us is out of range, return without doing anything.
  byte tmp;
  if(mode == REG_DM_PU) tmp = drivestates[pin.group].pullup;
  else if(mode == REG_DM_PD) tmp = drivestates[pin.group].pulldown;
  else if(mode == REG_DM_ODH) tmp = drivestates[pin.group].odhigh;
  else if(mode == REG_DM_ODL) tmp = drivestates[pin.group].odlow;
  else if(mode == REG_DM_STRONG) tmp = drivestates[pin.group].strong;
  else if(mode == REG_DM_SLOW) tmp = drivestates[pin.group].slow;
  else tmp = drivestates[pin.group].hiz;
  //Modify the single pin
  tmp |= 1 << pin.pin;

  /* I don't like the next statement, but it doesn't seem to me that
   * anyone would want to constantly change drive modes.  If I'm wrong,
   * the math isn't that hard to switch out so that we have local state
   * math done, rather than hitting up the i2c bus.
   */
  drivestates[pin.group] = __getDrive(pin.group); //Re-read the groups drive states
  //Call the port
  __portSelect(pin.group);
  //Call __driveSelect
  __driveSelect(tmp,mode);
}

/* driveSelectAll(byte mode)
 * This method sets the drive mode for all pins
 */
void CY8C95X0::driveSelectAll(byte mode)
{
  for(int i=0; i < group_c; i++)
  {
    __portSelect(i);
    __driveSelect(0xFF,mode);
  }
}
/* driveSelectGroup(uint8_t group, byte mode)
 * This method sets the drive mode for a whole pin group
 */
void CY8C95X0::driveSelectGroup(uint8_t group, byte mode)
{
  __portSelect(group);
  __driveSelect(0xFF,mode);
}
/* driveMode(uint8_t pin, byte mode)
 * This method changes drive mode for a single pin
 */
void CY8C95X0::driveMode(uint8_t pin, byte mode)
{
  _driveSelectPin(pinTranslate(pin),mode);
}


  /***************************
   * Pin direction functions *
   ***************************/


/* __getPortDirection(uint8_t group)
 * This method returns a byte of the pinstates for a pin group
 */
byte CY8C95X0::__getPortDirection(uint8_t group)
{
  byte tmp = 0;
  __portSelect(group);
  rawWrite(1, REG_PIN_DIR);
  Wire.requestFrom(address, uint8_t(1));
  if(Wire.available()) tmp = Wire.read();
  return tmp;
}
/* __pinDirection(byte pins)
 * This method writes the pin directions of a previously selected port
 */
void CY8C95X0::__pinDirection(byte pins)
{
  rawWrite(2,REG_PIN_DIR,pins);
}
/* _pinMode(pin_t pin, boolean mode)
 * This method sets a particular pin as an input or an output
 */
void CY8C95X0::_pinMode(pin_t pin, boolean mode)
{
  if(mode == INPUT) pinstates[pin.group] |= (1 << pin.pin); //If high, set the bit
  else pinstates[pin.group] &= ~(1 << pin.pin); //If low, clear the bit
  __portSelect(pin.group); //Call the port
  __pinDirection(pinstates[pin.group]); //Set the data
}
/* pinMode(uint8_t pinIn, boolean mode)
 * This method sets the direction/mode of a pin (INPUT/OUTPUT)
 * -This replicates the functionality of arduino
 */
void CY8C95X0::pinMode(uint8_t pinIn, boolean mode)
{
  _pinMode(pinTranslate(pinIn), mode);
}
/* pinMode(byte groupIn, byte pinIn, boolean mode)
 * This method sets the direction/mode of a group/pin combination (INPUT/OUTPUT)
 */
void CY8C95X0::pinMode(uint8_t group, uint8_t pin, boolean mode)
{
  pin_t pinIn = {group,pin};
  _pinMode(pinIn, mode);
}


  /*****************
   * I/O Functions *
   *****************/


/* __digitalH(byte command, byte pins)
 * This method writes pin data to the device for output
 */
void CY8C95X0::__digitalH(byte command, byte pins)
{
  rawWrite(2,command,pins);
}
/* _digitalRead(pin_t pin)
 * This method reads the state of a pin
 */
boolean CY8C95X0::_digitalRead(pin_t pin)
{
  byte tmp;
  rawWrite(1, REG_GI0 + pin.group);
  Wire.requestFrom(address, uint8_t(1));
  if(Wire.available()) tmp = Wire.read();
  if((tmp & (1 << pin.pin)) >= 1) return HIGH;
  else return LOW;
}
/* digitalRead(uint8_t pin)
 * This method reads the state of a pin
 * -Identical to the arduino function
 */
boolean CY8C95X0::digitalRead(uint8_t pin)
{
  return _digitalRead(pinTranslate(pin));
}
/* digitalRead(uint8_t group, uint8_t pin)
 * This method reads the state of a pin
 * -Almost like the arduino method, just easier for this chip, because the
 * datasheet doesn't relate port numbers the same way.
 */
boolean CY8C95X0::digitalRead(uint8_t group, uint8_t pin)
{
  pin_t tmp = {group, pin};
  return _digitalRead(tmp);
}

/* _digitalWrite(pin_t pin, boolean mode)
 * This method writes a state to a pin
 */
void CY8C95X0::digitalWrite(pin_t pin, boolean mode)
{
  if(mode == HIGH) pinstates[pin.group] |= 1 << pin.pin; //If high, set the bit
  if(mode == LOW) pinstates[pin.group] &= ~(1 << pin.pin); //If low, clear the bit
  __digitalH((REG_GO0 + pin.group),pinstates[pin.group]);
}
/* digitalWrite(uint8_t pin, boolean mode)
 * This method writes a state to a pin
 */
void CY8C95X0::digitalWrite(uint8_t pinIn, boolean mode)
{
  _digitalWrite(pinTranslate(pin),mode);
}
/* digitalWrite(uint8_t group, uint8_t pin, boolean mode)
 * This method writes a state to a pin
 */
void CY8C95X0::digitalWrite(byte group, byte pin, boolean mode)
{
  pin_t pinIn = {group,pin};
  _digitalWrite(pinIn,mode);
}
/* digitalWrite(boolean mode)
 * This method writes a state to every pin on the device
 */
void CY8C95X0::digitalWrite(boolean mode)
{
  //There is a faster way to do this, but for now, just iterate writes.
  for(int i = 0; i < group_c; i++)
  {
    if(mode == HIGH) __digitalH(i,ALL_PINS);
    else __digitalH(i,NO_PINS);
  }
  
}
