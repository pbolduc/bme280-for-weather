#ifndef __BME280_H__
#define __BME280_H__

#include "Arduino.h"

#include <Wire.h>

#define BME280_OK (0)

#define BME280_MODE_SLEEP 0b00
#define BME280_MODE_FORCED 0b01
#define BME280_MODE_NORMAL 0b11

#define BME280_FILTER_OFF 0
#define BME280_FILTER_2 2

struct BME280_Reading
{
  // temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
  int32_t temperature;

  // humidity in %RH as unsigned 32 bit integer in Q22. 10 format (22 integer and 10 fractional bits).
  // Output value of “47445” represents 47445/1024 = 46.333 %RH
  uint32_t humidity;

  // pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits)
  // Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
  uint32_t pressure;
};

class BME280
{

    public:
        BME280(void);

        uint8_t begin(TwoWire *wire, Serial_ *debug = NULL);

       /**
         * Reads the BME280 sensor
         */
        uint8_t readSensor(BME280_Reading *reading);

       /**
         * Sets the data acquisition options for the BME280 sensor.
         * 
         * @param temperature the temperature over sample: 0, 1, 2, 4, 8 or 16
         * @param humdity the humdity over sample: 0, 1, 2, 4, 8 or 16
         * @param pressure the pressure over sample: 0, 1, 2, 4, 8 or 16
         * @param mode the requested mode: 0 (sleep), 1 or 2 (forced) or 3 (normal)
         */
        uint8_t setDataAcquisitionOptions(uint8_t temperatureOverSample, uint8_t humdityOverSample, uint8_t pressureOverSample, uint8_t mode = BME280_MODE_SLEEP);

        /**
         * Sets the rate, filter and interface options
         */
        uint8_t setConfiguration(uint8_t standby, uint8_t filter, bool enableThreeWireSPI = false);

        uint8_t getMode();
        uint8_t setMode(uint8_t mode);

        /**
         * Resets the device using the complete power-on-reset procedure
         */
        void reset();
    
    private:
        // Used to hold the calibration constants.  These are used
        // by the driver as measurements are being taking
        struct CompensationCalibration
        {
          public:
          uint16_t dig_T1;
          int16_t dig_T2;
          int16_t dig_T3;
          
          uint16_t dig_P1;
          int16_t dig_P2;
          int16_t dig_P3;
          int16_t dig_P4;
          int16_t dig_P5;
          int16_t dig_P6;
          int16_t dig_P7;
          int16_t dig_P8;
          int16_t dig_P9;
          
          uint8_t dig_H1;
          int16_t dig_H2;
          uint8_t dig_H3;
          int16_t dig_H4;
          int16_t dig_H5;
          int8_t dig_H6;
        };

        // converts the over sample 
        uint8_t convertOverSampleValue(uint8_t value);

        void readRegisterRegion(uint8_t *outputPointer , uint8_t offset, uint8_t length);

        uint8_t readRegister(uint8_t offset);
        void writeRegister(uint8_t offset, uint8_t dataToWrite);

        void readCalibrationData();
        void parseTemperaturePressureCalibrationData(const uint8_t *reg_data);
        void parseHumidityCalibrationData(const uint8_t *reg_data);

        /**
         * Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
         * 
         * @param reg_data the 
         * @param t_fine pointer to variable to hold the fine resolution temperature value
         */
        int32_t compensateTemperature(const uint8_t *reg_data, int32_t *t_fine);

        /**
         * Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
         * Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
         * 
         * @param pressure the uncompensated 20 bit value read from registers
         * @param t_fine fine resolution temperature value returned from compensateTemperature()
         */
        uint32_t compensatePressure(const uint8_t *reg_data, int32_t t_fine);

        /**
         * Returns humidity in %RH as unsigned 32 bit integer in Q22. 10 format (22 integer and 10 fractional bits).
         * Output value of “47445” represents 47445/1024 = 46.333 %RH
         * 
         * @param humidity the uncompensated 16 bit value read from registers
         * @param t_fine fine resolution temperature value returned from compensateTemperature()
         */
        uint32_t compensateHumidity(const uint8_t *reg_data, int32_t t_fine);
        
        TwoWire *wire;
        CompensationCalibration calibration;
        uint8_t I2CAddress;

        Serial_ *debug;

};

#endif
