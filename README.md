
# BME280 for Weather

This is an opinionated library targeting the BME280 sensor for weather sensing only. At present, this is a commit 
of my [Platform IO](https://platformio.org/) project. The main BME280 library code is in the [lib](lib) directory.

## Status
This driver is still experimental.  I have 4 [BME280 Breakout](https://lowpowerlab.com/shop/product/185) sensors
from [LowPowerLab](https://lowpowerlab.com/) which I am testing on the [MoteinoM0 R1](https://lowpowerlab.com/shop/product/184)

**Note**: I do not have any affiliation or relationship with [LowPowerLab](https://lowpowerlab.com/). I just like their quality products.

## Assumptions
* Your project is running on a battery
* Your project is transmitting your results over a RF transmitter
* Your project is only transmitting results when values change more than a certain threshold or minimal amount of time, ie every 15 minutes.

## Reading and Intrepting the Results

Note that the type `BME280_Reading` holds the temperature, humidity and pressure readings. Since the micro-controller 
I am using, Corext M0+, does not have native floating point operations, doing floating point math is relatively expensive.
Therefore the measurement values do not represent real world reportable values. The reason for this is in typical battery 
operated weather sensor, you will only publish your readings if they have changed more than some threshold. Publishing of 
results is assumed to be a power expensive operation due to turning on RF radio transmitter.

### Temperature

Temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.  To get the real temperature,
divide by 100.0.

### Humidity

Humidity in %RH as unsigned 32 bit integer in Q22. 10 format (22 integer and 10 fractional bits). 
Output value of “47445” represents 47445/1024 = 46.333 %RH.  To get the real humidity values divide by 1024.0

### Pressure

Pressure in Pa as unsigned 32 bit integer. Output value of "96386"  96386 Pa = 963.86 hPa


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

