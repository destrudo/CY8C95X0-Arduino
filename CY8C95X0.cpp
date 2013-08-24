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

CY8C95X0::CY8C95X0()
{
  Wire.begin();
}

/* If you want to use the default constructor so that you aren't allocating the class (Intended for non-dynamic setups)
 * after ALL of your begin() functions are called, you'll need to call this.  MAKE SURE that if you want to dump serial
 * data over the UART, that you call Serial.begin() BEFORE this is called. */
 //chip is the rev of the chip (20,40,60) and A is the complete set of A's
// ex: (20,011) for A0 and A1 set on a 20 i/o chip
void CY8C95X0::begin(uint8_t chip, uint8_t A)
{
  address = CY8C95X0_ADDR;
  
  /* Address handling */
  if(A)
  {
	if( A <= 31 ) address |= A; //Supposing you're setting those bits that are below A5, just OR the address with your address
	else address = A; //If you're not, you might as well just set the whole address yourself, and I'll save myself some slight logic.
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
  
  /* Handle all the addresses */
  
  __getConfig(); //Update the current config
}


//chip is the rev of the chip (20,40,60) and A is the complete set of A's
// ex: (20,011) for A0 and A1 set on a 20 i/o chip
CY8C95X0::CY8C95X0(uint8_t chip, uint8_t A)
{
  CY8C95X0::begin(chip, A);
}

/* GENERAL FUNCTIONS BEGIN (Internal math and computation) */

/* To make life easier, this function translates a number to a pin
 * the pin argument is /always/ a two member byte array, it will
 * Not function properly, and will probably kill the chip if called
 * with anything else
 * This is really messy
 */
pin_t CY8C95X0::pinTranslate(uint8_t pinIn)
{
  /* We need to compensate for group 2 */
  pin_t pin = {255,255};
  if((pinIn + 1) > pin_c) return pin; //If pin is outside values, exit.

  if(pinIn > 19) pinIn += 4; //Since port 2 only has a nibble, we need to jump ahead after that point
  pin.pin = byte(pinIn % 8); //set pin
  pin.group = byte(pinIn / 8); //set pin group
  return pin;
}

/* This function resets the device to the programmed defaults
 * When the class is first constructed, it runs this, in order
 * to purge any bad settings (Were there any)
 */
void CY8C95X0::resetChip()
{
  //Run command to reset chip
  rawWrite(2, REG_CMD, REG_CMD_RESTORE);
  rawWrite(2, REG_CMD, REG_CMD_RECONF);
}

/* This determines whether or not the port that was entered was valid
 * It is used by the higher level functions
 */
boolean CY8C95X0::validPort(byte port)
{
   if( (port + 1) > group_c) return false;
   return true;
}

/* This is to simply reset the register pointer
 */
void CY8C95X0::resetRegister()
{
  Wire.beginTransmission(address);
  Wire.write((uint8_t)0);
  Wire.endTransmission();
}

/* This function accepts a pin number and returns which pwm controller controls it */
int CY8C95X0::pinPWM(pin_t pin)
{
  /* Note: This matrix does not support the other chips, and will need to have cases built to do so */
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
  /*This is a mess, but for ease of use... */
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

/* LOW LEVEL FUNCTIONS BEGIN */
/* rawWrite(int cmds, ...)
 * arguments:
 * -cmds is number of commands to be sent
 * -varable arguments will be written in order
 * ex: rawWrite(2, REG_PORT_SEL, 0x00) would select group port 0
 *
 * This is pretty low level, but if you need to do something I didn't
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

/* Get pwm config settings */
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

/* get drive states for a group */
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

/* Get port pwm status */
byte CY8C95X0::__getPortPWM(uint8_t group)
{
  byte tmp;
  __portSelect(group);
  rawWrite(1, REG_SEL_PWM_PORT_OUT);
  Wire.requestFrom(address, uint8_t(1));
  if(Wire.available()) tmp = Wire.read();
  return tmp;
}

/* Get port input/output mode */
byte CY8C95X0::__getPortDirection(uint8_t group)
{
  byte tmp = 0;
  __portSelect(group);
  rawWrite(1, REG_PIN_DIR);
  Wire.requestFrom(address, uint8_t(1));
  if(Wire.available()) tmp = Wire.read();
  return tmp;
}

/* Get port interrupts */
byte CY8C95X0::__interrupt(uint8_t group)
{
  __portSelect(group);
  rawWrite(1, REG_INT_STAT_0 + group);
  Wire.requestFrom(address, uint8_t(1));
  if(Wire.available()) return Wire.read();
  return 0;
}

/* Get port interrupts for a group */
byte CY8C95X0::_interrupt(uint8_t group)
{
  if(group >= group_c) return 0x00;
  return __interrupt(group);
}

/* The following 3 functions return a boolean which is the
 * interrupt status on a particular pin
 */
boolean CY8C95X0::interrupt(pin_t pin)
{
  boolean tmp = false;
  tmp = __interrupt(pin.group) & (1 << pin.pin);
  return tmp;
}

/* by a group/pin combo */
boolean CY8C95X0::interrupt(uint8_t group, uint8_t pin)
{
  pin_t tmp = {group, pin};
  return (interrupt(tmp));
}

/* by raw pin number */
boolean CY8C95X0::interrupt(uint8_t pin)
{
  return interrupt(pinTranslate(pin));
}

byte CY8C95X0::__getInterruptMask(uint8_t group)
{
  __portSelect(group);
  rawWrite(1, REG_INT_MASK);
  Wire.requestFrom(address, uint8_t(1));
  if(Wire.available()) return Wire.read();
  return 0x00;
}

byte CY8C95X0::_getInterruptMask(uint8_t group)
{
  if(group >= group_c) return 0x00;
  return __getInterruptMask(group);
}
boolean CY8C95X0::getInterruptMask(pin_t pin)
{
  byte tmp = __getInterruptMask(pin.group);
  return (tmp & (1 << pin.pin));
//return (__getInterruptMask(pin.group) & (1 << pin.pin));
}
boolean CY8C95X0::getInterruptMask(uint8_t group, uint8_t pin)
{
  pin_t tmp = {group, pin};
  return getInterruptMask(tmp);
}
boolean CY8C95X0::getInterruptMask(uint8_t pin)
{
  return getInterruptMask(pinTranslate(pin));
}

/* Get port inversion states */
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
/* Get inversion states for a whole group */
byte CY8C95X0::_getInversionGroup(uint8_t group)
{
  if(group >= group_c) return 0x00;
  return __getInvStates(group);
}

boolean CY8C95X0::getInversion(pin_t pin)
{
  boolean tmp = false;
  tmp = __getInvStates(pin.group) & (1 << pin.pin);
  return tmp;
}

boolean CY8C95X0::getInversion(uint8_t group, uint8_t pin)
{
  pin_t tmp = {group,pin};
  return getInversion(tmp);
}

boolean CY8C95X0::getInversion(uint8_t pin)
{
  return getInversion(pinTranslate(pin));
}

/* High level pin invert (Just calls the register alone) */
void CY8C95X0::__invert(uint8_t pins)
{
  rawWrite(2,REG_INVERSION,pins);
}

/* This calls the required port select and __invert, as well as
 * validating the provided group */
void CY8C95X0::_invert(uint8_t group, uint8_t pins)
{
  __portSelect(group);
  __invert(pins);
}

/* This can either set all pins to inverting, turn off inverting,
 * or invert the current inverting 
 * The default is 0x01, All pins to be inverting
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

/* Set a pin to have inverse output */
/* There are modes allowed only in this function
 * which accepts a pin type.
 * 0 = turn off
 * 1 = Turn on //Default
 * everything else = toggle
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

void CY8C95X0::invert(uint8_t group, uint8_t pin)
{
  pin_t tmp = {group,pin};
  invert(tmp);
}

void CY8C95X0::invert(uint8_t pin)
{
  invert(pinTranslate(pin));
}

/*Toggle the inversion on a single pin */
void CY8C95X0::invertT(uint8_t pin)
{
  invert(pinTranslate(pin),0x02);
}
void CY8C95X0::invertT(uint8_t group, uint8_t pin)
{
  pin_t tmp = {group,pin};
  invert(tmp,0x02);
}
/* Turn inversion off for a pin */
void CY8C95X0::invertOff(uint8_t pin)
{
  invert(pinTranslate(pin),0x00);
}
void CY8C95X0::invertOff(uint8_t group, uint8_t pin)
{
  pin_t tmp = {group,pin};
  invert(tmp,0x00);
}


/* Get the divider value */
byte CY8C95X0::__getDivider()
{
  byte tmp;
  rawWrite(1, REG_PROG_DIV);
  Wire.requestFrom(address, uint8_t(1));
  if(Wire.available()) tmp = Wire.read();
  return tmp;
}

/* Get output register values */
byte CY8C95X0::__getOutput(uint8_t group)
{
  byte tmp;
  rawWrite(1, REG_GO0 + group);
  Wire.requestFrom(address, uint8_t(1));
  if(Wire.available()) tmp = Wire.read();
  return tmp;
}

/* Get input register values */
byte CY8C95X0::__getInput(uint8_t group)
{
  byte tmp;
  rawWrite(1, (0x00 + group));
  Wire.requestFrom(address, uint8_t(1));
  if(Wire.available()) tmp = Wire.read();
  return tmp;
}

/* This function populates all of the controller data (pin modes, pwm modes, intterupt, pwm config) */
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

/* This is a low-level function for handling port selection as detailed in the datasheet */
void CY8C95X0::__portSelect(byte port)
{
  if(!validPort(port)) return; //If the port specified is out of our range, exit
  rawWrite(2,REG_PORT_SEL,port);
}

/* This is a low-level function for handling pwm enabling/disabling
 * It writes a byte of pins to the device
 */
void CY8C95X0::__pwmSelect(byte pins)
{
  rawWrite(2,REG_SEL_PWM_PORT_OUT,pins); //Send the command
}

//This is the pwm select register command
void CY8C95X0::__pwmConfigSelect(byte controller)
{
  rawWrite(2,REG_SEL_PWM,controller);
}
/* PWM Clock source setting register */
void CY8C95X0::__pwmClockSel(byte pwmController)
{
  rawWrite(2,REG_CONF_PWM,pwmController);
}
/* PWM Period setting register */
void CY8C95X0::__pwmConfigPeriod(byte period)
{
  rawWrite(2,REG_PERI_PWM,period);
}
/* PWM Pulse Width setting register */
void CY8C95X0::__pwmConfigPulseWidth(byte pulsewidth)
{
  rawWrite(2,REG_PW_PWM,pulsewidth);
}
/* This is the non-complex pwm config function.
 * The period is a completely optional argument
 * since so few people will realistically need
 * a higher frequency than 130hz.  But incase
 * You need some math pointers,
 * 1khz = 0x22 to the register
 * 142hz = 0xF0, 2khz = 0x11, 435hz = 0x4F
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
/* This is the complex pwm config function.
 * This accepts a clock source, a period, a pulse width, and a circuit.
 * There's no default anything here, they must all be set
 * Read the data sheet if you intend to use this, otherwise you'll regret
 * it just as I have.
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



/* This is the low-level pin mode function.
 * each bytes bits correspond to the pins on that port
 * a high bit means that pin will be an input, and of course
 * a low bit means the pin will be an output.
 */
void CY8C95X0::__pinDirection(byte pins)
{
  rawWrite(2,REG_PIN_DIR,pins);
}

/* This is a low-level function for handling pin driving modes across a whole group
 */
//void CY8C95X0::__driveSelect(byte pins, byte mode)
void CY8C95X0::__driveSelect(byte pins, byte mode)
{
  rawWrite(2,mode,pins);
}

/* Handles a drive select on a single pin */
/* We should really make it so that mode is the actual Register
 * so that we don't waste more space with the case scenarios
 */
void CY8C95X0::_driveSelectPin(pin_t pin, byte mode)
{
  byte tmp;
  switch(mode)
  {
    case 0:
      tmp = drivestates[pin.group].pullup;
      break;
    case 1:
      tmp = drivestates[pin.group].pulldown;
      break;
    case 2:
      tmp = drivestates[pin.group].odhigh;
      break;
    case 3:
      tmp = drivestates[pin.group].odlow;
      break;
    case 4:
      tmp = drivestates[pin.group].strong;
      break;
    case 5:
      tmp = drivestates[pin.group].slow;
      break;
    case 6:
      tmp = drivestates[pin.group].hiz;
      break;
  }
  //Modify the single pin
  tmp |= 1 << pin.pin;
  
  drivestates[pin.group] = __getDrive(pin.group); //Re-read the groups drive states
  //Call the port
  __portSelect(pin.group);
  //Call __driveSelect
  __driveSelect(tmp,mode);
}

/* This sets the drive mode, something very important if you're doing PWM
 * mode can be:
 * 0: Resistive pullup
 * 1: Resistive pulldown
 * 2: High open drain
 * 3: low open drain
 * 4: strong drive
 * 5: strong and slow drive
 * 6: High impedance
 * Arguments are <Bus (0-7)>, <Pin (0-7)> and 
 */
void CY8C95X0::driveSelectAll(byte mode)
{
  for(int i=0; i < group_c; i++)
  {
    __portSelect(i);
    __driveSelect(0xFF,mode);
  }
}

/* Handles drive selection setting for a whole group at once */
void CY8C95X0::driveSelectGroup(uint8_t port, byte mode)
{
  //call port group
  __portSelect(port);
  __driveSelect(0xFF,mode);
}

/* Changes driving mode for a single pin */
void CY8C95X0::driveMode(uint8_t pin, byte mode)
{
  _driveSelectPin(pinTranslate(pin),mode);
}

/* Programmable divider setting
 * Using this without understanding is a bad idea, it can mess with things in a bad way.
 * Its intent is for using the 00000100b setting on the pwm config to achieve custom frequencies
 * on the controller.  Problem is, it applies to all controllers.  So setting it will mess with
 * the other pwms.
 *
 * Here in the case that someone wants to use the programmable clock for some much-more-accurate
 * frequencies.
 *
 * I include no sanity check, you get to do that.
 */
void CY8C95X0::__pwmConfigDivider(byte divider)
{
  rawWrite(2,REG_PROG_DIV,divider);
}

/* Digital Handler
 * Writes two bytes, where the first is the command (Starting point register) and the second is the
 * byte of pin status to be set
 */
void CY8C95X0::__digitalH(byte command, byte pins)
{
  rawWrite(2,command,pins);
}

/* This low-level call enables or disables a particular pin */
/* pin_t in this function is used to specify a /particular/ pin, not all of the pins */
void CY8C95X0::_digitalWrite(pin_t pin, boolean mode)
{
  if(mode == HIGH) pinstates[pin.group] |= 1 << pin.pin; //If high, set the bit
  if(mode == LOW) pinstates[pin.group] &= ~(1 << pin.pin); //If low, clear the bit
  __digitalH((REG_GO0 + pin.group),pinstates[pin.group]); //Digitalwrite doesn't need to select any port, just directly write away.
}



/* Sets pins as inputs or outputs, accepts a pin type */
void CY8C95X0::_pinMode(pin_t pin, boolean mode)
{
  if(mode == INPUT) pinstates[pin.group] |= (1 << pin.pin); //If high, set the bit
  else pinstates[pin.group] &= ~(1 << pin.pin); //If low, clear the bit
  __portSelect(pin.group); //Call the port
  __pinDirection(pinstates[pin.group]); //Set the data
}

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

/* USER-LEVEL FUNCTIONS BEGIN */

/* turn pwm on or off on a single pin */
void CY8C95X0::pwmSelect(uint8_t pinIn, boolean mode)
{
  pin_t pin_v = pinTranslate(pinIn);
  _pwmSelect(pin_v, mode);
}
/* Turn pwm on or off for a single pin */
void CY8C95X0::pwmSelect(uint8_t group, uint8_t pinIn, boolean mode)
{
  pin_t pinOut = {group,pinIn};
  _pwmSelect(pinOut, mode);
}
/* Sets all pins pwm mode on or off */
void CY8C95X0::pwmSelect(boolean mode)
{
  for(int i = 0; i < group_c; i++)
  {
    __portSelect(i);
    if(mode == HIGH) __pwmSelect(ALL_PINS);
    else __pwmSelect(NO_PINS);
  }
}

/* Modify everything */
void CY8C95X0::pwmConfig(byte circuit, byte clock, byte period, byte pw)
{
  //Sanity check the values
  if(circuit >= pwm_c) return;
  if(pw >= period) return;
  _pwmConfig(circuit, clock, period, pw);
}
/* Modify duty cycle only */
void CY8C95X0::pwmConfig(byte circuit, byte duty)
{
  if(circuit >= pwm_c) return; //If the circuit requested is higher than that on our chip, exit.
  if(duty >= pwmconf[circuit].period) return; //Pulse width, according to the datasheet, should always be less than the period by at least 1
  _pwmConfig(circuit, duty);
}

/* lower level digital read */
boolean CY8C95X0::_digitalRead(pin_t pin)
{
  byte tmp;
  rawWrite(1, REG_GI0 + pin.group);
  Wire.requestFrom(address, uint8_t(1));
  if(Wire.available()) tmp = Wire.read();
  if((tmp & (1 << pin.pin)) >= 1) return HIGH;
  else return LOW;
}

//Exactly like the arduino method
boolean CY8C95X0::digitalRead(uint8_t pin)
{
  return _digitalRead(pinTranslate(pin));
}

/* Almost like the arduino method, just easier for this chip, because the
 * datasheet doesn't relate port numbers the same way.
 */
boolean CY8C95X0::digitalRead(uint8_t group, uint8_t pin)
{
  pin_t tmp = {group, pin};
  return _digitalRead(tmp);
}
/* This works quite the same way as digitalwrite */
void CY8C95X0::digitalWrite(uint8_t pinIn, boolean mode)
{
  _digitalWrite(pinTranslate(pinIn),mode);
}
//In case you want to set it the more datasheet translatable way
void CY8C95X0::digitalWrite(byte groupIn, byte pinIn, boolean mode)
{
  pin_t pin_v = {groupIn,pinIn};
  _digitalWrite(pin_v,mode);
}    
//This is for writing to every pin on the chip
void CY8C95X0::digitalWrite(boolean mode)
{
  //There is a faster way to do this, but for now, just iterate writes.
  for(int i = 0; i < group_c; i++)
  {
    if(mode == HIGH) __digitalH(i,ALL_PINS);
    else __digitalH(i,NO_PINS);
  }
  
}

//Set pin as input or output the same as arduino
void CY8C95X0::pinMode(uint8_t pinIn, boolean mode)
{
  _pinMode(pinTranslate(pinIn), mode);
}

void CY8C95X0::pinMode(byte groupIn, byte pinIn, boolean mode)
{
  pin_t pin = {groupIn,pinIn};
  _pinMode(pin, mode);
}

/* This is the function to use to configure the pwm circuits */
/* It isn't a great idea to mess with unless you know what you're doing */
//    void pwmConfig(byte pwmController, byte clock, byte period, byte pulse, byte divider)
//    {
//      if( pulse >= period ) return;
//      if(period == 0x00) return;
//      if(divider == 0x00) return;
//    }

/* Same as analogWrite */
void CY8C95X0::analogWrite(pin_t pin, uint8_t value)
{
  _pwmConfig(pinPWM(pin),value);
}
void CY8C95X0::analogWrite(byte group, byte pin, uint8_t value)
{
  pin_t tmp = {group, pin};
  _pwmConfig(pinPWM(tmp), value);
}
void CY8C95X0::analogWrite(uint8_t pin, uint8_t value)
{
  _pwmConfig(pinPWM(pinTranslate(pin)),value);
}
