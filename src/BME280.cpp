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
#include <math.h>


#define BME280_REGISTER_DIG_T1           0x88
#define BME280_REGISTER_DIG_T2           0x8A
#define BME280_REGISTER_DIG_T3           0x8C

#define BME280_REGISTER_DIG_P1           0x8E
#define BME280_REGISTER_DIG_P2           0x90
#define BME280_REGISTER_DIG_P3           0x92
#define BME280_REGISTER_DIG_P4           0x94
#define BME280_REGISTER_DIG_P5           0x96
#define BME280_REGISTER_DIG_P6           0x98
#define BME280_REGISTER_DIG_P7           0x9A
#define BME280_REGISTER_DIG_P8           0x9C
#define BME280_REGISTER_DIG_P9           0x9E

#define BME280_REGISTER_DIG_H1           0xA1
#define BME280_REGISTER_DIG_H2           0xE1
#define BME280_REGISTER_DIG_H3           0xE3
#define BME280_REGISTER_DIG_H4           0xE4
#define BME280_REGISTER_DIG_H5           0xE5
#define BME280_REGISTER_DIG_H6           0xE7

#define BME280_REGISTER_CHIPID           0xD0
#define BME280_REGISTER_VERSION          0xD1
#define BME280_REGISTER_SOFTRESET        0xE0

#define BME280_REGISTER_CAL26            0xE1  /* R calibration stored in 0xE1-0xF0 */

#define BME280_REGISTER_CONTROLHUMID     0xF2
#define BME280_REGISTER_STATUS           0XF3
#define BME280_REGISTER_CONTROL          0xF4
#define BME280_REGISTER_CONFIG           0xF5
#define BME280_REGISTER_PRESSUREDATA     0xF7
#define BME280_REGISTER_TEMPDATA         0xFA
#define BME280_REGISTER_HUMIDDATA        0xFD


BME280::BME280(uint8_t _addr, TwoWire & _i2c) :
  m_i2c(_i2c)
{
  m_i2c_addr = _addr;
}

bool BME280::begin()
{
  if (!m_i2c.isEnabled())
  {
    m_i2c.begin();
  }

  // check if sensor, i.e. the chip ID is correct
  if (read8(BME280_REGISTER_CHIPID) != 0x60)
    return false;

  // reset the device using soft-reset
  // this makes sure the IIR is off, etc.
  write8(BME280_REGISTER_SOFTRESET, 0xB6);

  // wait for chip to wake up.
  delay(300);

  read_calib_data(); // read trimming parameters, see DS 4.2.2

  set_sensor_mode(); // use defaults

  return true;
}

void BME280::set_sensor_mode(
    SensMode     _mode,
    SensSampling _temp_sampling,
    SensSampling _press_sampling,
    SensSampling _hum_sampling,
    SensFilter   _filter,
    SensStandby  _duration
  )
{
  m_meas_reg.mode   = _mode;
  m_meas_reg.osrs_t = _temp_sampling;
  m_meas_reg.osrs_p = _press_sampling;

  m_hum_reg.osrs_h  = _hum_sampling;

  m_conf_reg.filter = _filter;
  m_conf_reg.t_sb   = _duration;

  // you must make sure to also set REGISTER_CONTROL after setting the
  // CONTROLHUMID register, otherwise the values won't be applied (see DS 5.4.3)
  write8(BME280_REGISTER_CONTROLHUMID, m_hum_reg.get());
  write8(BME280_REGISTER_CONFIG,       m_conf_reg.get());
  write8(BME280_REGISTER_CONTROL,      m_meas_reg.get());
}

void BME280::make_forced_measurement()
{
  // If we are in forced mode, the BME sensor goes back to sleep after each
  // measurement and we need to set it to forced mode once at this point, so
  // it will take the next measurement and then return to sleep again.
  // In normal mode simply does new measurements periodically.

  if (m_meas_reg.mode == mode_forced)
  {
    // set to forced mode, i.e. "take next measurement"
    write8(BME280_REGISTER_CONTROL, m_meas_reg.get());

    // wait until measurement has been completed, otherwise we would read
    // the values from the last measurement
    while (read8(BME280_REGISTER_STATUS) & 0x08)
      delay(1);
  }
}

float BME280::get_temperature()
{
  int32_t adc_t = read24(BME280_REGISTER_TEMPDATA);
  if (adc_t == 0x800000) // value in case temp measurement was disabled
    return 0.0f;
  adc_t >>= 4;

  int32_t var1 = ((((adc_t >> 3) - ((int32_t)m_calib_data.dig_T1 << 1))) *
                  ((int32_t)m_calib_data.dig_T2)) >> 11;
  int32_t var2 = (((((adc_t >> 4) - ((int32_t)m_calib_data.dig_T1)) *
                  ((adc_t >> 4) - ((int32_t)m_calib_data.dig_T1))) >> 12) *
                  ((int32_t)m_calib_data.dig_T3)) >> 14;

  m_t_fine = var1 + var2;

  return (float)((m_t_fine * 5 + 128) >> 8) / 100.0f;
}

float BME280::get_pressure()
{
  get_temperature(); // must be done first to get t_fine

  int32_t adc_p = read24(BME280_REGISTER_PRESSUREDATA);
  if (adc_p == 0x800000) // value in case pressure measurement was disabled
    return 0.0f;
  adc_p >>= 4;

  int64_t var1, var2, p;

  var1 = ((int64_t)m_t_fine) - 128000;

  var2 = var1 * var1 * (int64_t)m_calib_data.dig_P6;
  var2 = var2 + ((var1 * (int64_t)m_calib_data.dig_P5) << 17);
  var2 = var2 + (((int64_t)m_calib_data.dig_P4) << 35);

  var1 = ((var1 * var1 * (int64_t)m_calib_data.dig_P3) >> 8) +
         ((var1 * (int64_t)m_calib_data.dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)m_calib_data.dig_P1) >> 33;

  if (var1 == 0)
    return 0.0f; // avoid exception caused by division by zero

  p = 1048576 - adc_p;
  p = (((p << 31) - var2) * 3125) / var1;

  var1 = (((int64_t)m_calib_data.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)m_calib_data.dig_P8) * p) >> 19;

  p = ((p + var1 + var2) >> 8) + (((int64_t)m_calib_data.dig_P7) << 4);

  return (float)p / 256.0f;
}

float BME280::get_humidity()
{
  get_temperature(); // must be done first to get t_fine

  int32_t adc_h = read16(BME280_REGISTER_HUMIDDATA);
  if (adc_h == 0x8000) // value in case humidity measurement was disabled
    return 0.0f;

  int32_t var1;

  var1 = (m_t_fine - ((int32_t)76800));

  var1 = (((((adc_h << 14) - (((int32_t)m_calib_data.dig_H4) << 20) -
             (((int32_t)m_calib_data.dig_H5) * var1)) + ((int32_t)16384)) >> 15) *
           (((((((var1 * ((int32_t)m_calib_data.dig_H6)) >> 10) *
               (((var1 * ((int32_t)m_calib_data.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
             ((int32_t)2097152)) * ((int32_t)m_calib_data.dig_H2) + 8192) >> 14));

  var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int32_t)m_calib_data.dig_H1)) >> 4));

  var1 = (var1 < 0) ? 0 : var1;
  var1 = (var1 > 419430400) ? 419430400 : var1;

  return  (float)(var1 >> 12) / 1024.0f;
}

float BME280::get_altitude()
{
  // Equation taken from BMP180 datasheet (page 16):
  //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

  // Note that using the equation from wikipedia can give bad results
  // at high altitude. See this thread for more information:
  //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

  const float atmospheric_pressure = get_pressure() / 100.0f;
  return 44330.0f * (1.0f - pow(atmospheric_pressure / m_sea_level, 0.1903f));
}

void BME280::set_sea_level(float _sea_level)
{
  m_sea_level = _sea_level;
}

void BME280::set_sea_level(float _altitude, float _pressure)
{
  // Equation taken from BMP180 datasheet (page 17):
  //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

  // Note that using the equation from wikipedia can give bad results
  // at high altitude. See this thread for more information:
  //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

  m_sea_level = _pressure / pow(1.0f - (_altitude / 44330.0f), 5.255f);
}

void BME280::write8(uint8_t _reg, uint8_t _value)
{
  m_i2c.beginTransmission(m_i2c_addr);
  m_i2c.write(_reg);
  m_i2c.write(_value);
  m_i2c.endTransmission();
}

uint8_t BME280::read8(uint8_t _reg)
{
  m_i2c.beginTransmission(m_i2c_addr);
  m_i2c.write(_reg);
  m_i2c.endTransmission();
  m_i2c.requestFrom(m_i2c_addr, 1);
  return m_i2c.read();
}

uint16_t BME280::read16(uint8_t _reg)
{
  m_i2c.beginTransmission(m_i2c_addr);
  m_i2c.write(_reg);
  m_i2c.endTransmission();
  m_i2c.requestFrom(m_i2c_addr, 2);
  return (m_i2c.read() << 8) | m_i2c.read();
}

uint32_t BME280::read24(uint8_t _reg)
{
  m_i2c.beginTransmission(m_i2c_addr);
  m_i2c.write(_reg);
  m_i2c.endTransmission();
  m_i2c.requestFrom(m_i2c_addr, 3);

  return (((m_i2c.read() << 8) | m_i2c.read()) << 8) | m_i2c.read();
}

uint16_t BME280::read16_LE(uint8_t _reg)
{
  uint16_t value = read16(_reg);
  return (value >> 8) | (value << 8);
}

void BME280::read_calib_data()
{
  // if chip is still reading calibration, delay
  for (uint8_t status = read8(BME280_REGISTER_STATUS); (status & (1 << 0)) != 0; )
  {
    delay(100);
    status = read8(BME280_REGISTER_STATUS);
  }

  m_calib_data.dig_T1 = (uint16_t)read16_LE(BME280_REGISTER_DIG_T1);
  m_calib_data.dig_T2 = ( int16_t)read16_LE(BME280_REGISTER_DIG_T2);
  m_calib_data.dig_T3 = ( int16_t)read16_LE(BME280_REGISTER_DIG_T3);

  m_calib_data.dig_P1 = (uint16_t)read16_LE(BME280_REGISTER_DIG_P1);
  m_calib_data.dig_P2 = ( int16_t)read16_LE(BME280_REGISTER_DIG_P2);
  m_calib_data.dig_P3 = ( int16_t)read16_LE(BME280_REGISTER_DIG_P3);
  m_calib_data.dig_P4 = ( int16_t)read16_LE(BME280_REGISTER_DIG_P4);
  m_calib_data.dig_P5 = ( int16_t)read16_LE(BME280_REGISTER_DIG_P5);
  m_calib_data.dig_P6 = ( int16_t)read16_LE(BME280_REGISTER_DIG_P6);
  m_calib_data.dig_P7 = ( int16_t)read16_LE(BME280_REGISTER_DIG_P7);
  m_calib_data.dig_P8 = ( int16_t)read16_LE(BME280_REGISTER_DIG_P8);
  m_calib_data.dig_P9 = ( int16_t)read16_LE(BME280_REGISTER_DIG_P9);

  m_calib_data.dig_H1 = (uint8_t)read8(BME280_REGISTER_DIG_H1);
  m_calib_data.dig_H2 = ( int16_t)read16_LE(BME280_REGISTER_DIG_H2);
  m_calib_data.dig_H3 = (uint8_t)read8(BME280_REGISTER_DIG_H3);
  m_calib_data.dig_H4 = (read8(BME280_REGISTER_DIG_H4)     << 4) | (read8(BME280_REGISTER_DIG_H4 + 1) & 0xf);
  m_calib_data.dig_H5 = (read8(BME280_REGISTER_DIG_H5 + 1) << 4) | (read8(BME280_REGISTER_DIG_H5)    >> 4);
  m_calib_data.dig_H6 = ( int8_t)read8(BME280_REGISTER_DIG_H6);
}



