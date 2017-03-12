#pragma once

/*
  Original library: https://github.com/adafruit/Adafruit_BME280_Library

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

// this will load the definition for common Particle variable types
#include <Particle.h>


#define BME280_BLINK_OFF       0
#define BME280_BLINK_2HZ       1
#define BME280_BLINK_1HZ       2
#define BME280_BLINK_HALFHZ    3


class BME280
{
public:

  struct CalibData
  {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;

    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;

    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;
  };

  enum SensMode
  {
    mode_sleep      = 0b00,
    mode_forced     = 0b01,
    mode_normal     = 0b11
  };

  enum SensSampling
  {
    sampling_none   = 0b000,
    sampling_x1     = 0b001,
    sampling_x2     = 0b010,
    sampling_x4     = 0b011,
    sampling_x8     = 0b100,
    sampling_x16    = 0b101
  };

  enum SensFilter
  {
    filter_off      = 0b000,
    filter_x2       = 0b001,
    filter_x4       = 0b010,
    filter_x8       = 0b011,
    filter_x16      = 0b100
  };

  // standby durations in ms
  enum SensStandby
  {
    standby_ms_0_5  = 0b000,
    standby_ms_10   = 0b110,
    standby_ms_20   = 0b111,
    standby_ms_62_5 = 0b001,
    standby_ms_125  = 0b010,
    standby_ms_250  = 0b011,
    standby_ms_500  = 0b100,
    standby_ms_1000 = 0b101
  };

  // Constructor: I2C address, I2C interface
  BME280(uint8_t _addr, TwoWire & _i2c);

  bool begin();

  // setup sensor with given parameters / settings
  void setup_sensor(
      SensMode     _mode           = mode_normal,
      SensSampling _temp_sampling  = sampling_x16,
      SensSampling _press_sampling = sampling_x16,
      SensSampling _hum_sampling   = sampling_x16,
      SensFilter   _filter         = filter_off,
      SensStandby  _duration       = standby_ms_0_5
    );

  // make a new measurement (only possible in forced mode)
  void make_forced_measurement();

  float get_temperature();

  float get_pressure();

  float get_humidity();

  float get_altitude();

  // Calculates the altitude (in meters) from the specified atmospheric pressure (in hPa), and sea-level pressure (in hPa).
  //    _sea_level  sea-level pressure in hPa
  void set_sea_level(float _sea_level);

  // Calculates the pressure at sea level (in hPa) from the specified altitude (in meters), and atmospheric pressure (in hPa).
  //    _altitude   altitude in meters
  //    _pressure   atmospheric pressure in hPa
  void set_sea_level(float _altitude, float _pressure);

protected:

  void write8(uint8_t _reg, uint8_t _value);

  uint8_t read8(uint8_t _reg);

  uint16_t read16(uint8_t _reg);

  uint32_t read24(uint8_t _reg);

  uint16_t read16_LE(uint8_t _reg); // little endian

  void read_calib_data();

  CalibData  m_calib_data;

  int32_t    m_t_fine;
  float      m_sea_level;

  // the config register
  struct config {
    // inactive duration (standby time) in normal mode
    // 000 = 0.5 ms
    // 001 = 62.5 ms
    // 010 = 125 ms
    // 011 = 250 ms
    // 100 = 500 ms
    // 101 = 1000 ms
    // 110 = 10 ms
    // 111 = 20 ms
    unsigned int t_sb : 3;

    // filter settings
    // 000 = filter off
    // 001 = 2x filter
    // 010 = 4x filter
    // 011 = 8x filter
    // 100 and above = 16x filter
    unsigned int filter : 3;

    // unused - don't set
    unsigned int none : 1;
    unsigned int spi3w_en : 1;

    unsigned int get()
    {
      return (t_sb << 5) | (filter << 3) | spi3w_en;
    }
  }          m_conf_reg;


  // the ctrl_meas register
  struct
  {
    // temperature oversampling
    // 000 = skipped
    // 001 = x1
    // 010 = x2
    // 011 = x4
    // 100 = x8
    // 101 and above = x16
    unsigned int osrs_t : 3;

    // pressure oversampling
    // 000 = skipped
    // 001 = x1
    // 010 = x2
    // 011 = x4
    // 100 = x8
    // 101 and above = x16
    unsigned int osrs_p : 3;

    // device mode
    // 00       = sleep
    // 01 or 10 = forced
    // 11       = normal
    unsigned int mode : 2;

    unsigned int get()
    {
      return (osrs_t << 5) | (osrs_p << 3) | mode;
    }
  }          m_meas_reg;


  // the ctrl_hum register
  struct
  {
    // unused - don't set
    unsigned int none : 5;

    // pressure oversampling
    // 000 = skipped
    // 001 = x1
    // 010 = x2
    // 011 = x4
    // 100 = x8
    // 101 and above = x16
    unsigned int osrs_h : 3;

    unsigned int get()
    {
      return (osrs_h);
    }
  }          m_hum_reg;

  uint8_t    m_i2c_addr;
  TwoWire &  m_i2c;
};





class Adafruit_BME280 {


    private:
        void readCoefficients(void);
        bool isReadingCalibration(void);
        uint8_t spixfer(uint8_t x);


        int32_t   _sensorID;


};


