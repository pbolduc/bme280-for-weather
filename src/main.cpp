#include <Arduino.h>
#include <Wire.h>

#include "LowPower.h"
#include <RTCZero.h>

#include "BME280.h"

BME280 sensor;
RTCZero rtc;

#define INTERVAL 60

void alarmMatch();
void standby(uint8_t minutes);

void setup() 
{
  SerialUSB.begin(115200);
  rtc.begin(); // initialize RTC 24H format
  rtc.setDate(1, 1, 20);
  rtc.setTime(0, 0, 0);

  Wire.begin();
  Wire.setClock(400000);

  sensor.begin(&Wire);

  // wake in one minute
  rtc.attachInterrupt(alarmMatch);
  rtc.enableAlarm(rtc.MATCH_MMSS);

  //standby((rtc.getMinutes()+1) % 60);
}

void alarmMatch()
{
  //digitalWrite(LED_BUILTIN, HIGH);
}

void standby(uint8_t minutes)
{
  rtc.setAlarmMinutes(minutes % 60);
  rtc.standbyMode();
}

void loop() {
  // put your main code here, to run repeatedly:
  uint8_t minutes = rtc.getMinutes();

  // reinitialize the serial after wake
  SerialUSB.begin(115200);

  BME280_Reading reading = { 0 };
  sensor.forceReadSensor(&reading);

  SerialUSB.print("Temperature: ");
  SerialUSB.print(reading.temperature / 100.0);
  SerialUSB.println(" C");

  SerialUSB.print("Humidity   : ");
  SerialUSB.print(reading.humidity / 1024.0);
  SerialUSB.println(" %");

  SerialUSB.print("Pressure   : ");
  SerialUSB.print(reading.pressure);
  SerialUSB.println(" Pa");

  SerialUSB.flush();

#ifdef BME280_MEASURE_READ_TIME
  SerialUSB.print("Measure Time   : ");
  SerialUSB.print(1.0 * sensor.totalTimeToStartMeasuring / sensor.measureCount);
  SerialUSB.print(" ");
  SerialUSB.println(1.0 * sensor.totalTimeToMeasure / sensor.measureCount);

  SerialUSB.print("Compensation Time   : ");
  SerialUSB.print(1.0 * sensor.totalTimeToCompensationTemperature / sensor.measureCount);
  SerialUSB.print("us (T), ");
  SerialUSB.print(1.0 * sensor.totalTimeToCompensationPressure / sensor.measureCount);
  SerialUSB.print("us (P), ");
  SerialUSB.print(1.0 * sensor.totalTimeToCompensationHumidity / sensor.measureCount);
  SerialUSB.print("us (H)");
  SerialUSB.println();
#endif

  // TODO: sleep instead of delay

#ifdef BME280_MEASURE_READ_TIME
  delay(1000 * 15); // wait 15 seconds
#else
  standby((minutes+1) % 60);
#endif

    SerialUSB.println("WAKEUP");
}
