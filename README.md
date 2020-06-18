
# BME280 for Weather

This is an opinionated library targeting the BME280 sensor for weather sensing only. At present, this is a commit 
of my Platform IO project. The main BME280 library code is in the [lib](lib) directory.

## Status
This driver is still experimental.  I have 4 [BME280 Breakout](https://lowpowerlab.com/shop/product/185) sensors
from [LowPowerLab](https://lowpowerlab.com/) which I am testing on the [MoteinoM0 R1](https://lowpowerlab.com/shop/product/184)

**Note**: I do not have any affiliation or relationship with [LowPowerLab](https://lowpowerlab.com/). I just like their quality products.

## Motivation
Many of the alternative libraries are designed to be generic and suit any of the recommended modes of operation, including,

* Weather Monitoring
* Humidity sensing
* Indoor navigation
* Gaming

I wanted a library that was as efficent and correct as possible for weather monitoring only. The sensor will be initialized
with the correct settings and the code will assume these settings do not need to be changed. Since we will always read with
1x pressure, 1x temperature and 1x humidity over sampling, trigging a forced read does not need to read the current register
mask to preserve the sampling values. The driver can just set these to '1'.

### Why not create pull requests for other established projects

There are well established existing libraries,

* [Adafruit_BME280_Library](https://github.com/adafruit/Adafruit_BME280_Library)
* [SparkFun_BME280_Arduino_Library](https://github.com/sparkfun/SparkFun_BME280_Arduino_Library)
* [Bosch Sensortec's BME280 pressure sensor driver](https://github.com/BoschSensortec/BME280_driver)

You may ask why don't I include these kind of changes to these projects. I considered doing that, but my changes would break existing 
users of these libraries.  Maintainers generally do not accept/approve pull requests that break their exising users.  
I have submitted a couple pull requests,

* [Avoid calling begin more than is required](https://github.com/sparkfun/SparkFun_BME280_Arduino_Library/pull/37)
* [Fix humidity uncompensated data based on data sheet memory map](https://github.com/BoschSensortec/BME280_driver/pull/76)

## Observed BME280 Sensor timings

Sleep -> Forced Mode: 

    0.81 ms to start measuring, measuring bit of the status register 0 -> 1
    7.50 ms to complete measurement cycle, measuring bit of the status register 1 -> 0

Compensating the calculations from internal ADC values:

Temperature: 3.77 us
Pressure   : 9.81 us (32 bit mode), 43.32 us in (64 bit mode)
Humidity   : 4.73 us

## Alternative Libraries

* [Adafruit_BME280_Library](https://github.com/adafruit/Adafruit_BME280_Library)
* [SparkFun_BME280_Arduino_Library](https://github.com/sparkfun/SparkFun_BME280_Arduino_Library)
* [Bosch Sensortec's BME280 pressure sensor driver](https://github.com/BoschSensortec/BME280_driver)

