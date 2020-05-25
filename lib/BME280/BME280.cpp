#include "BME280.h"

#define BME280_INVALID_OVERSAMPLE 0xff


// registers
#define BME280_CHIP_ID_REG				0xD0 // Chip ID, can be read as soon as the device is finished power-on-reset
#define BME280_RST_REG					0xE0 // Softreset Reg
#define BME280_CTRL_HUMIDITY_REG		0xF2 // Ctrl Humidity Reg
#define BME280_STAT_REG					0xF3 // Status Reg
#define BME280_CTRL_MEAS_REG			0xF4 // Ctrl Measure Reg

#define BME280_TEMP_PRESS_CALIB_DATA_ADDR   0x88
#define BME280_HUMIDITY_CALIB_DATA_ADDR     0xE1

#define BME280_TEMP_PRESS_CALIB_DATA_LEN    26
#define BME280_HUMIDITY_CALIB_DATA_LEN      7

#define BME280_P_T_H_DATA_LEN               8

#define BME280_DATA_ADDR                  0xF7

/**\name Macro to combine two 8 bit data's to form a 16 bit data */
#define BME280_CONCAT_BYTES(msb, lsb)            (((uint16_t)msb << 8) | (uint16_t)lsb)

BME280::BME280(void) {
    this->I2CAddress = 0x77;
}

uint8_t BME280::begin(TwoWire *wire, Serial_ *debug)
{
    this->wire = wire;
    this->debug = debug;

    // check chip id
    uint8_t chipId = readRegister(BME280_CHIP_ID_REG);

    readCalibrationData();

    return BME280_OK;
}

uint8_t BME280::readSensor(BME280_Reading *reading)
{
    uint8_t reg_data[BME280_P_T_H_DATA_LEN] = { 0 };

    readRegisterRegion(reg_data, BME280_DATA_ADDR, BME280_P_T_H_DATA_LEN);

    int32_t t_fine = 0;
    reading->temperature = compensateTemperature(reg_data, &t_fine);
    reading->pressure = compensatePressure(reg_data, t_fine);
    reading->humidity = compensateHumidity(reg_data, t_fine);

    return BME280_OK;
}

uint8_t BME280::setDataAcquisitionOptions(uint8_t temperatureOverSample, uint8_t humdityOverSample, uint8_t pressureOverSample, uint8_t mode)
{
    // register  | address | bit7 | bit6 | bit5 | bit4 | bit3 | bit2 | bit1 | bit0 |
    // --------- | ------- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- |
    // ctrl_meas | 0xF4    |       osrs_t       |       osrs_p       |    mode     |
    // status    | 0xf3    |                           | meas |             | upd  |
    // ctrl_hum  | 0xf2    |                      
    //
    uint8_t osrs_t = convertOverSampleValue(temperatureOverSample);
    if (osrs_t == BME280_INVALID_OVERSAMPLE) {
        return 1; // invalid temperature over sample
    }

    uint8_t osrs_h = convertOverSampleValue(humdityOverSample);
    if (osrs_h == BME280_INVALID_OVERSAMPLE) {
        return 2; // invalid humdity over sample
    }

    uint8_t osrs_p = convertOverSampleValue(pressureOverSample);
    if (osrs_p == BME280_INVALID_OVERSAMPLE) {
        return 3;  // invalid pressure over sample
    }

    if (mode > 0b11) {
        return 4; // invalid mode
    }

    // ctrl_meas needs to be written after changing ctrl_hum for the changes to be effective

    uint8_t ctrl_hum = readRegister(BME280_CTRL_HUMIDITY_REG);
    uint8_t ctrl_meas = readRegister(BME280_CTRL_MEAS_REG);

    ctrl_hum = (ctrl_hum & 0b11111000) | osrs_h;
    ctrl_meas = (osrs_t << 5) | (osrs_p << 2) | (mode & 0b00000011);

    writeRegister(BME280_CTRL_HUMIDITY_REG, ctrl_hum);
    writeRegister(BME280_CTRL_MEAS_REG, ctrl_meas);

    return BME280_OK;
}

uint8_t BME280::convertOverSampleValue(uint8_t value)
{
    switch (value)
    {
        case 0: return 0;
        case 1: return 1;
        case 2: return 2;
        case 4: return 3;
        case 8: return 4;
        case 16: return 5;

        // invalid
        default: return BME280_INVALID_OVERSAMPLE;
    }
}

uint8_t BME280::getMode()
{
    uint8_t ctrl_meas = readRegister(BME280_CTRL_MEAS_REG);
    return (ctrl_meas & 0b00000011);
}

uint8_t BME280::setMode(uint8_t mode)
{
    if (mode > 0b11) 
    {
        return 1; // invalid mode
    }

    uint8_t ctrl_meas = readRegister(BME280_CTRL_MEAS_REG);

    ctrl_meas = (ctrl_meas & 0b11111100) | mode;
    writeRegister(BME280_CTRL_MEAS_REG, ctrl_meas);

    return 0;
}

void BME280::reset()
{
    /* 0xB6 is the soft reset command */
    writeRegister(BME280_RST_REG, 0xB6);

    // start up time is 2 ms - time to first communication
    delay(2);

    uint8_t im_update = 0;
    uint8_t retry = 5;

    // wait for the im_update to reset to 0 after
    // NVM data has been copied to image registers
    do
    {
        im_update = readRegister(BME280_STAT_REG) & 0b00000001;
    } while (im_update && retry--);

    if (im_update == 0)
    {

    }
    
}

// private functions

uint8_t BME280::readRegister(uint8_t offset)
{
    uint8_t result = 0;

    wire->beginTransmission(I2CAddress);
    wire->write(offset);
    wire->endTransmission();

    wire->requestFrom(I2CAddress, 1);
    while (wire->available()) 
    {
        result = wire->read();
    }

    return result;
}

void BME280::readRegisterRegion(uint8_t *outputPointer , uint8_t offset, uint8_t length)
{
    uint8_t i = 0;
    char c = 0;

    // The Wire library implementation uses a 32 byte buffer, therefore any communication
    // should be within this limit. Exceeding bytes in a single transmission will just be
    // dropped

    wire->beginTransmission(I2CAddress);
    wire->write(offset);
    uint8_t rc = wire->endTransmission(false); // dont send stop cause we are going to re-read

    // endTransmission Errors:
    //  0 : Success
    //  1 : Data too long
    //  2 : NACK on transmit of address
    //  3 : NACK on transmit of data
    //  4 : Other error
    
    wire->requestFrom(I2CAddress, length);
    while (wire->available() && (i < length)) 
    {
        c = wire->read();
        *outputPointer = c;
        outputPointer++;
        i++;
    }
}

void BME280::writeRegister(uint8_t offset, uint8_t dataToWrite)
{
    wire->beginTransmission(I2CAddress);
    wire->write(offset);
    wire->write(dataToWrite);
    uint8_t rc = wire->endTransmission();

    // endTransmission Errors:
    //  0 : Success
    //  1 : Data too long
    //  2 : NACK on transmit of address
    //  3 : NACK on transmit of data
    //  4 : Other error
}

void BME280::readCalibrationData()
{
    // calibration data is found in two register ranges
    // 0x88 - 0xA1
    // 0xE1 - 0xF0
    // is is faster to read these ranges at once instead of reading each byte

    uint8_t calibration_data[BME280_TEMP_PRESS_CALIB_DATA_LEN] = { 0 };

    readRegisterRegion(calibration_data, BME280_TEMP_PRESS_CALIB_DATA_ADDR, BME280_TEMP_PRESS_CALIB_DATA_LEN);
    parseTemperaturePressureCalibrationData(calibration_data);

    readRegisterRegion(calibration_data, BME280_HUMIDITY_CALIB_DATA_ADDR, BME280_HUMIDITY_CALIB_DATA_LEN);
    parseHumidityCalibrationData(calibration_data);
}

void BME280::parseTemperaturePressureCalibrationData(const uint8_t *reg_data)
{
    // temperature
    calibration.dig_T1 = BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
    calibration.dig_T2 = (int16_t)BME280_CONCAT_BYTES(reg_data[3], reg_data[2]);
    calibration.dig_T3 = (int16_t)BME280_CONCAT_BYTES(reg_data[5], reg_data[4]);

    // pressure
    calibration.dig_P1 = BME280_CONCAT_BYTES(reg_data[7], reg_data[6]);
    calibration.dig_P2 = (int16_t)BME280_CONCAT_BYTES(reg_data[9], reg_data[8]);
    calibration.dig_P3 = (int16_t)BME280_CONCAT_BYTES(reg_data[11], reg_data[10]);
    calibration.dig_P4 = (int16_t)BME280_CONCAT_BYTES(reg_data[13], reg_data[12]);
    calibration.dig_P5 = (int16_t)BME280_CONCAT_BYTES(reg_data[15], reg_data[14]);
    calibration.dig_P6 = (int16_t)BME280_CONCAT_BYTES(reg_data[17], reg_data[16]);
    calibration.dig_P7 = (int16_t)BME280_CONCAT_BYTES(reg_data[19], reg_data[18]);
    calibration.dig_P8 = (int16_t)BME280_CONCAT_BYTES(reg_data[21], reg_data[20]);
    calibration.dig_P9 = (int16_t)BME280_CONCAT_BYTES(reg_data[23], reg_data[22]);
    
    // humidity
    calibration.dig_H1 = reg_data[25];
}

void BME280::parseHumidityCalibrationData(const uint8_t *reg_data)
{
    int16_t dig_h4_lsb;
    int16_t dig_h4_msb;
    int16_t dig_h5_lsb;
    int16_t dig_h5_msb;

    calibration.dig_H2 = (int16_t)BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
    calibration.dig_H3 = reg_data[2];

    dig_h4_msb = (int16_t)(int8_t)reg_data[3] * 16;
    dig_h4_lsb = (int16_t)(reg_data[4] & 0x0F);
    calibration.dig_H4 = dig_h4_msb | dig_h4_lsb;

    dig_h5_msb = (int16_t)(int8_t)reg_data[5] * 16;
    dig_h5_lsb = (int16_t)(reg_data[4] >> 4);

    calibration.dig_H5 = dig_h5_msb | dig_h5_lsb;
    calibration.dig_H6 = (int8_t)reg_data[6];
}

int32_t BME280::compensateTemperature(const uint8_t *reg_data, int32_t *t_fine)
{
    // compute temperature the uncompensated 20 bit value read from registers
    int32_t adc_temperature = ((uint32_t)reg_data[3] << 12) | ((uint32_t)reg_data[4] << 4) | ((uint32_t)reg_data[5] >> 4);

    // Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.

    int32_t var1;
    int32_t var2;
    int32_t temperature;
    int32_t temperature_min = -4000;
    int32_t temperature_max = 8500;

    var1 = (int32_t)((adc_temperature / 8) - ((int32_t)calibration.dig_T1 * 2));
    var1 = (var1 * ((int32_t)calibration.dig_T2)) / 2048;
    var2 = (int32_t)((adc_temperature / 16) - ((int32_t)calibration.dig_T1));
    var2 = (((var2 * var2) / 4096) * ((int32_t)calibration.dig_T3)) / 16384;
    
    *t_fine = var1 + var2;
    temperature = (*t_fine * 5 + 128) / 256;

    if (temperature < temperature_min)
    {
        temperature = temperature_min;
    }
    else if (temperature > temperature_max)
    {
        temperature = temperature_max;
    }

    return temperature;
}

uint32_t BME280::compensatePressure(const uint8_t *reg_data, int32_t t_fine)
{
    /* Store the parsed register values for pressure data */
    int32_t adc_pressure = ((uint32_t)reg_data[0] << 12) | ((uint32_t)reg_data[1] << 4) | ((uint32_t)reg_data[2] >> 4);
   
    // Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
    // Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa

    int64_t var1;
    int64_t var2;
    int64_t var3;
    int64_t var4;
    uint32_t pressure;
    uint32_t pressure_min = 3000000;
    uint32_t pressure_max = 11000000;

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calibration.dig_P6;
    var2 = var2 + ((var1 * (int64_t)calibration.dig_P5) * 131072);
    var2 = var2 + (((int64_t)calibration.dig_P4) * 34359738368);
    var1 = ((var1 * var1 * (int64_t)calibration.dig_P3) / 256) + ((var1 * ((int64_t)calibration.dig_P2) * 4096));
    var3 = ((int64_t)1) * 140737488355328;
    var1 = (var3 + var1) * ((int64_t)calibration.dig_P1) / 8589934592;

    /* To avoid divide by zero exception */
    if (var1 != 0)
    {
        var4 = 1048576 - adc_pressure;
        var4 = (((var4 * INT64_C(2147483648)) - var2) * 3125) / var1;
        var1 = (((int64_t)calibration.dig_P9) * (var4 / 8192) * (var4 / 8192)) / 33554432;
        var2 = (((int64_t)calibration.dig_P8) * var4) / 524288;
        var4 = ((var4 + var1 + var2) / 256) + (((int64_t)calibration.dig_P7) * 16);
        pressure = (uint32_t)(((var4 / 2) * 100) / 128);

        if (pressure < pressure_min)
        {
            pressure = pressure_min;
        }
        else if (pressure > pressure_max)
        {
            pressure = pressure_max;
        }
    }
    else
    {
        pressure = pressure_min;
    }

    return pressure;
}

uint32_t BME280::compensateHumidity(const uint8_t *reg_data, int32_t t_fine)
{
    int32_t adc_humidity = ((uint32_t)reg_data[6] << 8) | ((uint32_t)reg_data[7]);

    // Returns humidity in %RH as unsigned 32 bit integer in Q22. 10 format (22 integer and 10 fractional bits).
    // Output value of “47445” represents 47445/1024 = 46.333 %RH

    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t var4;
    int32_t var5;
    uint32_t humidity;
    uint32_t humidity_max = 102400;

    var1 = t_fine - ((int32_t)76800);
    var2 = (int32_t)(adc_humidity * 16384);
    var3 = (int32_t)(((int32_t)calibration.dig_H4) * 1048576);
    var4 = ((int32_t)calibration.dig_H5) * var1;
    var5 = (((var2 - var3) - var4) + (int32_t)16384) / 32768;
    var2 = (var1 * ((int32_t)calibration.dig_H6)) / 1024;
    var3 = (var1 * ((int32_t)calibration.dig_H3)) / 2048;
    var4 = ((var2 * (var3 + (int32_t)32768)) / 1024) + (int32_t)2097152;
    var2 = ((var4 * ((int32_t)calibration.dig_H2)) + 8192) / 16384;
    var3 = var5 * var2;
    var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
    var5 = var3 - ((var4 * ((int32_t)calibration.dig_H1)) / 16);
    var5 = (var5 < 0 ? 0 : var5);
    var5 = (var5 > 419430400 ? 419430400 : var5);
    humidity = (uint32_t)(var5 / 4096);

    if (humidity > humidity_max)
    {
        humidity = humidity_max;
    }

    return humidity;
}