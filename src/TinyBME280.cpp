/*****************************************************/
/*                                                   */
/*                    TinyBME280                     */
/*      (c) Yvan Régeard - All rights reserved       */
/*                                                   */
/* Licensed under the MIT license. See LICENSE file  */
/* in the project root for full license information. */
/*                                                   */
/*****************************************************/



// Includes
#include <Arduino.h>
#include <Wire.h>
#include "TinyBME280.h"



// BME280 public functions

  // Constructor
  BME280::BME280(uint8_t address):
    m_address(address)
  {
  }



  // Initialize device
  bool BME280::initialize(uint8_t sda_pin,uint8_t scl_pin,uint32_t bus_frequency)
  {
    // Release SDA and SCL pins
    pinMode(sda_pin,INPUT);
    pinMode(scl_pin,INPUT);

    // Initialize I2C bus
    if (!Wire.begin(sda_pin,scl_pin,bus_frequency)) return false;

    // Read compensation values block #1 (24 bytes)
    Wire.beginTransmission(m_address);
    Wire.write(0x88);
    if (Wire.endTransmission(false)) return false;
    if (Wire.requestFrom(m_address,(uint8_t) 24)!=24) return false; 

      // Temperature compensation values (little endian)
      m_dig_T1=(Wire.read()<<0)|(Wire.read()<<8);
      m_dig_T2=(Wire.read()<<0)|(Wire.read()<<8);
      m_dig_T3=(Wire.read()<<0)|(Wire.read()<<8);

      // Pressure calibration (little endian)
      m_dig_P1=(Wire.read()<<0)|(Wire.read()<<8);
      m_dig_P2=(Wire.read()<<0)|(Wire.read()<<8);
      m_dig_P3=(Wire.read()<<0)|(Wire.read()<<8);
      m_dig_P4=(Wire.read()<<0)|(Wire.read()<<8);
      m_dig_P5=(Wire.read()<<0)|(Wire.read()<<8);
      m_dig_P6=(Wire.read()<<0)|(Wire.read()<<8);
      m_dig_P7=(Wire.read()<<0)|(Wire.read()<<8);
      m_dig_P8=(Wire.read()<<0)|(Wire.read()<<8);
      m_dig_P9=(Wire.read()<<0)|(Wire.read()<<8);

    // End transmission
    if (Wire.endTransmission()) return false;

    // Read compensation values block #2 (1 byte)
    Wire.beginTransmission(m_address);
    Wire.write(0xA1);
    if (Wire.endTransmission(false)) return false;
    if (Wire.requestFrom(m_address,(uint8_t) 1)!=1) return false;

      // Humidity compensation values (little endian)
      m_dig_H1=Wire.read();

    // End transmission
    if (Wire.endTransmission()) return false;

    // Read compensation values block #3 (6 bytes)
    Wire.beginTransmission(m_address);
    Wire.write(0xE1);
    if (Wire.endTransmission(false)) return false;
    if (Wire.requestFrom(m_address,(uint8_t) 6)!=6) return false;

      // Humidity compensation values (little endian)
      m_dig_H2=(Wire.read()<<0)|((int16_t) Wire.read()<<8);
      m_dig_H3=Wire.read();
      m_dig_H4=((int16_t) Wire.read()<<4);
      uint8_t temp_dig=Wire.read();
      m_dig_H4|=temp_dig&0x0F;
      m_dig_H5=(temp_dig>>4)|((int16_t) Wire.read()<<4);
      m_dig_H6=Wire.read();

    // End transmission
    if (Wire.endTransmission()) return false;

    // Read settings (3 bytes)
    Wire.beginTransmission(m_address);
    Wire.write(0xF2);
    if (Wire.endTransmission(false)) return false;
    if (Wire.requestFrom(m_address,(uint8_t) 3)!=3) return false;

      // Humidity oversampling
      m_osrs_h=Wire.read()&0x07;

      // Unused
      Wire.read();

      // Mode, pressure and temperature oversampling
      uint8_t register_value=Wire.read();
      m_mode=register_value&0x03;
      m_osrs_p=(register_value>>2)&0x07;
      m_osrs_t=(register_value>>5)&0x07;

    // End transmission
    if (Wire.endTransmission()) return false;

    // Return: OK
    return true;
  }



  // Settings
  bool BME280::set_mode(uint8_t mode) { m_mode=mode; return write_i2c_register(0xF4,(read_i2c_register(0xF4)&0xFC)|m_mode); }
  bool BME280::set_standby_time(uint8_t t_sb) { return write_i2c_register(0xF5,(read_i2c_register(0xF5)&0x1F)|t_sb); }
  bool BME280::set_filter(uint8_t filter) { return write_i2c_register(0xF5,(read_i2c_register(0xF5)&0xE3)|filter); }
  bool BME280::set_humidity_oversampling(uint8_t osrs_h) { m_osrs_h=osrs_h; return write_i2c_register(0xF2,osrs_h); }
  bool BME280::set_pressure_oversampling(uint8_t osrs_p) { m_osrs_p=osrs_p; return write_i2c_register(0xF4,(read_i2c_register(0xF4)&0xE3)|(osrs_p<<2)); }
  bool BME280::set_temperature_oversampling(uint8_t osrs_t) { m_osrs_t=osrs_t; return write_i2c_register(0xF4,(read_i2c_register(0xF4)&0x1F)|(osrs_t<<5)); }



  // Reset device
  bool BME280::reset()
  {
    // Write reset register
    return write_i2c_register(0xE0,0xB6);
  }



  // Read values
  bool BME280::read()
  {
    // Check mode
    if (m_mode!=BME280_MODE_NORMAL)
    {
      // Set forced mode to start measurement
      set_mode(BME280_MODE_FORCED);

      // Wait for measure completion
      delay(ceil(1.25+(2.3*m_osrs_t)+(2.3*m_osrs_p+0.575)+(2.3*m_osrs_h+0.575)));
    }

    // Read uncompensated values block (8 bytes)
    Wire.beginTransmission(m_address);
    Wire.write(0xF7);
    if (Wire.endTransmission(false)) return false;
    if (Wire.requestFrom(m_address,(uint8_t) 8)!=8) return false;

      // Uncompensated values
      int32_t uncompensated_pressure=(Wire.read()<<12)|(Wire.read()<<4)|(Wire.read()>>4);
      int32_t uncompensated_temperature=(Wire.read()<<12)|(Wire.read()<<4)|(Wire.read()>>4);
      int32_t uncompensated_humidity=(Wire.read()<<8)|(Wire.read()<<0);

    // End transmission
    if (Wire.endTransmission()) return false;

    // Get compensated values
    m_temperature=get_temperature(uncompensated_temperature);
    m_humidity=get_humidity(uncompensated_humidity);
    m_pressure=get_pressure(uncompensated_pressure);

    // Return: OK
    return true;
  }



// BME280 private functions

  // Get humidity value
  float BME280::get_humidity(int32_t uncompensated_humidity)
  {
    int32_t v_x1_u32r=m_fine_temperature-76800;
    v_x1_u32r=(((((((int32_t) uncompensated_humidity)<<14)-(((int32_t) m_dig_H4)<<20)-(m_dig_H5*v_x1_u32r))+16384)>>15)*(((((((v_x1_u32r*m_dig_H6)>>10)*(((v_x1_u32r*m_dig_H3)>>11)+32768))>>10)+2097152)*((int32_t) m_dig_H2)+8192)>>14));
    v_x1_u32r=(v_x1_u32r-(((((v_x1_u32r>>15)*(v_x1_u32r>>15))>>7)*m_dig_H1)>>4));
    v_x1_u32r=((v_x1_u32r<0)?0:v_x1_u32r);
    v_x1_u32r=((v_x1_u32r>419430400)?419430400:v_x1_u32r);
    return (v_x1_u32r>>12)/1024.0;
  }



  // Get pressure value
  float BME280::get_pressure(int32_t uncompensated_pressure)
  {
    int64_t var1=m_fine_temperature-128000;
    int64_t var2=var1*var1*m_dig_P6;
    var2+=((var1*m_dig_P5)<<17);
    var2+=(((int64_t) m_dig_P4)<<35);
    var1=((var1*var1*m_dig_P3)>>8)+((var1*m_dig_P2)<<12);
    var1=(((((((int64_t) 1)<<47)+var1))*m_dig_P1)>>33);
    if (var1==0) return 0.0;
    int64_t p=1048576-uncompensated_pressure;
    p=(((p<<31)-var2)*3125)/var1;
    var1=(((m_dig_P9*(p>>13)*(p>>13)))>>25);
    var2=(((m_dig_P8*p))>>19);
    p=((p+var1+var2)>>8)+(((int64_t) m_dig_P7)<<4);
    return p/25600.0;
  }



  // Get temperature value
  float BME280::get_temperature(int32_t uncompensated_temperature)
  {
    int32_t var1=((((uncompensated_temperature>>3)-(((uint32_t) m_dig_T1)<<1)))*m_dig_T2)>>11;
    int32_t var2=(((uncompensated_temperature>>4)-m_dig_T1*((uncompensated_temperature>>4)-m_dig_T1))>>12)*(m_dig_T3>>14);
    m_fine_temperature=var1+var2;
    return ((m_fine_temperature*5+128)>>8)/100.0;
  }



  // Read register
  uint8_t BME280::read_i2c_register(uint8_t reg)
  {
    // Read register byte
    Wire.beginTransmission(m_address);
    Wire.write(reg);
    if (Wire.endTransmission(false)) return 0xFF;
    if (Wire.requestFrom(m_address,(uint8_t) 1)!=1) return 0xFF;

      // Register value
      uint8_t result=Wire.read();

    // End transmission
    if (Wire.endTransmission()) return 0xFF;

    // Return: value
    return result;
  }



  // Write register
  bool BME280::write_i2c_register(uint8_t reg,uint8_t value)
  {
    // Write register byte
    Wire.beginTransmission(m_address);
    Wire.write(reg);
    Wire.write(value);

    // End transmission
    return (Wire.endTransmission()==0);
  }
