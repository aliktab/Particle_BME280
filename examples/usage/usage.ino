/*
  Copyright (C) 2014 Alik <aliktab@gmail.com> All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/

#include "BME280.h"
BME280 sensor(0x76, Wire);

void setup()
{
  sensor.begin();

  sensor.set_sensor_mode(
      BME280::mode_forced,
      BME280::sampling_x16,
      BME280::sampling_x16,
      BME280::sampling_x16,
      BME280::filter_off,
      BME280::standby_ms_0_5
    );

  Serial.begin();
}

void loop()
{
  sensor.make_forced_measurement();

  Serial.println(String::format("t: %f, p: %f, h: %f",
    sensor.get_temperature(), sensor.get_pressure(), sensor.get_humidity()));

  delay(1000);
}
