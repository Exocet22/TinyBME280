/*****************************************************/
/*                                                   */
/*                    TinyBME280                     */
/*      (c) Yvan Régeard - All rights reserved       */
/*                                                   */
/* Licensed under the MIT license. See LICENSE file  */
/* in the project root for full license information. */
/*                                                   */
/*****************************************************/
#ifndef _TINY_BME280_H_
#define _TINY_BME280_H_



// Constants

  // Default I2C address
  #define BME280_DEFAULT_I2C_ADDRESS      0x76

  // Mode
  #define BME280_MODE_SLEEP               0x00
  #define BME280_MODE_FORCED              0x01
  #define BME280_MODE_NORMAL              0x03

  // Standby time
  #define BME280_STANDBY_0_5              0x00
  #define BME280_STANDBY_62_5             0x20
  #define BME280_STANDBY_125              0x40
  #define BME280_STANDBY_250              0x60
  #define BME280_STANDBY_500              0x80
  #define BME280_STANDBY_1000             0xA0
  #define BME280_STANDBY_10               0xC0
  #define BME280_STANDBY_20               0xE0

  // Filtering
  #define BME280_FILTERING_OFF            0x00
  #define BME280_FILTERING_x2             0x04
  #define BME280_FILTERING_x4             0x08
  #define BME280_FILTERING_x8             0x0C
  #define BME280_FILTERING_x16            0x10

  // Oversampling
  #define BME280_OVERSAMPLING_OFF         0x00
  #define BME280_OVERSAMPLING_x1          0x01
  #define BME280_OVERSAMPLING_x2          0x02
  #define BME280_OVERSAMPLING_x4          0x03
  #define BME280_OVERSAMPLING_x8          0x04
  #define BME280_OVERSAMPLING_x16         0x05



// BME280 class definition
class BME280
{
  // Public atributes
  public:

    // Values
    float m_humidity;
    float m_pressure;
    float m_temperature;



  // Private atributes
  private:

    // Module address
    uint8_t m_address;

    // Settings
    uint8_t m_mode;
    uint8_t m_osrs_h,m_osrs_p,m_osrs_t;

    // Compensation values

      // Humidity
      uint8_t m_dig_H1,m_dig_H3;
      int16_t m_dig_H2,m_dig_H4,m_dig_H5;
      int8_t m_dig_H6;

      // Pressure
      uint16_t m_dig_P1;
      int16_t m_dig_P2,m_dig_P3,m_dig_P4,m_dig_P5,m_dig_P6,m_dig_P7,m_dig_P8,m_dig_P9;

      // Temperature
      uint16_t m_dig_T1;
      int16_t m_dig_T2,m_dig_T3;

    // Fine temperature
    int32_t m_fine_temperature;



  // Public methods
  public:

    // Constructor
    BME280(uint8_t address=BME280_DEFAULT_I2C_ADDRESS);

    // Initialize
    bool initialize(uint8_t sda_pin,uint8_t scl_pin,uint32_t bus_frequency);

    // Settings
    bool set_mode(uint8_t mode);
    bool set_standby_time(uint8_t t_sb);
    bool set_filter(uint8_t filter);
    bool set_humidity_oversampling(uint8_t osrs_h);
    bool set_pressure_oversampling(uint8_t osrs_p);
    bool set_temperature_oversampling(uint8_t osrs_t);

    // Reset device
    bool reset();

    // Read values
    bool read();



  // Private methods
  private:

    // Get compensated values
    float get_humidity(int32_t uncompensated_humidity);
    float get_pressure(int32_t uncompensated_pressure);
    float get_temperature(int32_t uncompensated_temperature);

    // I2C
    uint8_t read_i2c_register(uint8_t reg);
    bool write_i2c_register(uint8_t reg,uint8_t value);
};



#endif
