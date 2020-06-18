#ifndef __BME280_H__
#define __BME280_H__

#include "Arduino.h"

//#define BME280_MEASURE_READ_TIME

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

  // pressure in Pa as unsigned 32 bit integer
  // Output value of "96386"  96386 Pa = 963.86 hPa
  uint32_t pressure;
};

class BME280
{

    public:
        BME280(void);

        uint8_t begin(TwoWire *wire);

        /**
         * Performs a force read by
         */
        uint8_t forceReadSensor(BME280_Reading *reading);

        /**
         * Resets the device using the complete power-on-reset procedure
         */
        void reset();

#ifdef BME280_MEASURE_READ_TIME
        unsigned long totalTimeToStartMeasuring = 0;
        unsigned long totalTimeToMeasure = 0;
        unsigned long measureCount = 0;

        unsigned long totalTimeToCompensationTemperature = 0;
        unsigned long totalTimeToCompensationHumidity = 0;
        unsigned long totalTimeToCompensationPressure = 0;
#endif

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

        void initializeSensor();

        // converts the over sample 
        //uint8_t convertOverSampleValue(uint8_t value);

        void readRegisterRegion(uint8_t *outputPointer , uint8_t offset, uint8_t length);

        uint8_t readRegister(uint8_t offset);
        void writeRegister(uint8_t offset, uint8_t dataToWrite);

        void readCalibrationData();
        void parseTemperaturePressureCalibrationData(const uint8_t *reg_data);
        void parseHumidityCalibrationData(const uint8_t *reg_data);

        /**
         * Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
         * 
         * @param reg_data the data buffer for the read measurements
         * @param t_fine pointer to variable to hold the fine resolution temperature value
         */
        int32_t compensateTemperature(const uint8_t *reg_data, int32_t *t_fine);

        /**
         * Returns pressure in Pa as unsigned 32 bit integer
         * 
         * @param reg_data the data buffer for the read measurements
         * @param t_fine fine resolution temperature value returned from compensateTemperature()
         */
        uint32_t compensatePressure(const uint8_t *reg_data, int32_t t_fine);

        /**
         * Returns humidity in %RH as unsigned 32 bit integer in Q22. 10 format (22 integer and 10 fractional bits).
         * Output value of “47445” represents 47445/1024 = 46.333 %RH
         * 
         * @param reg_data the data buffer for the read measurements
         * @param t_fine fine resolution temperature value returned from compensateTemperature()
         */
        uint32_t compensateHumidity(const uint8_t *reg_data, int32_t t_fine);
        
        TwoWire *wire;
        CompensationCalibration calibration;
        uint8_t I2CAddress;
};

#endif
