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

  // Measure humidity, pressure and temperature
  g_bme280.set_humidity_oversampling(BME280_OVERSAMPLING_x1);
  g_bme280.set_pressure_oversampling(BME280_OVERSAMPLING_x1);
  g_bme280.set_temperature_oversampling(BME280_OVERSAMPLING_x1);
}



// Main loop
void loop()
{
  // Read BME280 values
  if (g_bme280.read())
  {
    // Debug trace
    Serial.println("BME280 values:");
    Serial.printf("Humidity:    %.2f%%\n",g_bme280.m_humidity);
    Serial.printf("Pressure:    %.2f hPa\n",g_bme280.m_pressure);
    Serial.printf("Temperature: %.2f °C\n",g_bme280.m_temperature);
    Serial.println();
  }

  // Delay
  delay(10000);
}
