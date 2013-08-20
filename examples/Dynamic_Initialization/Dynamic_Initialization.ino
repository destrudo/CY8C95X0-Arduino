/* Software License:

  Dynamic initialization example of CY8C95X0 library
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

/* This is an example where our class is instantiated globally
 * But not constructed until after Wire and Serial are begin'd
 *
 * Why?:
 * -You might want to dynamically enable chips
 *
 * Warning:
 * -If you're running an old version of arduino, your chip
 * 'driver' might not be happy with doing this.
 *
 * This example initializes the a pwm controller and switches
 * on pwm for pin #0.  It then dims until it can't dim anymore
 * and the value resets.
 */

#include <Wire.h>
#include <CY8C95X0.h>
#include <CY8C95X0_BASE.h>


uint8_t place = 254; // Our 'place', not that it is less than the period
CY8C95X0 * test; //Our un-init'd global

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  test = new CY8C95X0(60); //Construct the type (Default is a 60 pin)
  test->pwmSelect(0,HIGH); //Switch pwm on
  test->pwmConfig(0,place);
}

void loop()
{
  test->analogWrite(0,place); //Write the data
  place--; //lower the value
  if(place <= 0) //If the value is 0
    place = 254; //Reset it
  delay(10);
}


