#include <Arduino.h>
#include <Wire.h>

#include "BME280.h"

BME280 sensor;


void setup() {

    SerialUSB.begin(115200);
    Wire.begin();

    sensor.begin(&Wire, &SerialUSB);


}

void loop() {
  // put your main code here, to run repeatedly:
}
