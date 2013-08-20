/* Software License:

  Period modification example of CY8C95X0 library
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

/* This is an example much like the basics with the pin that dims
 * Slowly.  Just this time, the period is modified.
 */
#include <Wire.h>
#include <CY8C95X0.h>
#include <CY8C95X0_BASE.h>

uint8_t place = 120;
CY8C95X0 test;

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  test.begin(60); //Begin with a 60 pin
  test.pwmSelect(0,HIGH);
  test._pwmConfig(test.pinPWM(test.pinTranslate(0)),place,121);
}

void loop()
{
  test.analogWrite(0,place);
  place--;
  if(place <= 0)
    place = 120;
  delay(10);
}
