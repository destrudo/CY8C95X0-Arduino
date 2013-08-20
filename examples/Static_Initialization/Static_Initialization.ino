/* Software License:

  Static initialization example of CY8C95X0 library
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

/* This is an example where our class is globally called, but
 * the begin() function has not yet run
 *
 * Why?:
 * -You might want to make your code look more like the rest
 * of arduino
 * -You want to make extremely large static arrays filled with
 * this class so that you can run 128 chips at once
 *
 * This example initializes the a pwm controller and switches
 * on pwm for pin #0.  It then dims until it can't dim anymore
 * and the value resets.
 */

#include <Wire.h>
#include <CY8C95X0.h>
#include <CY8C95X0_BASE.h>

uint8_t place = 254;
CY8C95X0 test;

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  test.begin(60); //Begin with a 60 pin
  test.pwmSelect(0,HIGH);
  test.pwmConfig(0,place);
}

void loop()
{
  test.analogWrite(0,place);
  place--;
  if(place <= 0)
    place = 254;
  delay(10);
}
