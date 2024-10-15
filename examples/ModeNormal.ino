/*****************************************************/
/*                                                   */
/*                Test for TinyBME280                */
/*      (c) Yvan Régeard - All rights reserved       */
/*                                                   */
/* Licensed under the MIT license. See LICENSE file  */
/* in the project root for full license information. */
/*                                                   */
/*****************************************************/



// Includes
#include <Arduino.h>
#include "TinyBME280.h"



// Constants

  // I2C
  #define I2C_SDA_PIN                     32
  #define I2C_SCL_PIN                     17
  #define I2C_BUS_FREQUENCY               100E3



// Global variables

  // BME280 module
  BME280 g_bme280;



// Initialization
void setup()
{
  // Wait for stable signal
  delay(150);

  // Initialize debug traces
  Serial.begin(115200);
  Serial.println("\n");

  // Debug trace
  Serial.println("****** Test application for TinyBME280 Library ******");

  // Initialize BME280 module
  g_bme280.initialize(I2C_SDA_PIN,I2C_SCL_PIN,I2C_BUS_FREQUENCY);

  // Settings for indoor navigation
  g_bme280.set_mode(BME280_MODE_NORMAL);
  g_bme280.set_standby_time(BME280_STANDBY_0_5);
  g_bme280.set_filter(BME280_FILTERING_x16);

  // Measure humidity, pressure and temperature
  g_bme280.set_humidity_oversampling(BME280_OVERSAMPLING_x1);
  g_bme280.set_pressure_oversampling(BME280_OVERSAMPLING_x16);
  g_bme280.set_temperature_oversampling(BME280_OVERSAMPLING_x2);
}



// Main loop
void loop()
{
  // Every 40 ms (25 Hz)
  static uint32_t timeout=0;
  if (micros()>timeout)
  {
    timeout=micros()+40000;

    // Read BME280 values
    if (g_bme280.read())
    {
      // Debug trace
      Serial.printf("%.2f%% | %.2f hPa | %.2f °C\n",g_bme280.m_humidity,g_bme280.m_pressure,g_bme280.m_temperature);
    }
  }
}
