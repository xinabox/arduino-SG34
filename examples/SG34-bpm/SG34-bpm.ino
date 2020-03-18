#include <Wire.h>
#include <xCore.h>
#include "xSG34.h"

xSG34 particleSensor;

#define Serial SerialUSB

void setup()
{
  Serial.begin(115200);
  Serial.println("Initializing...");

  // Initialize sensor
  if (!particleSensor.begin()) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");
}

void loop()
{
  
  particleSensor.poll();
  Serial.println("BPM");
  Serial.println(particleSensor.getBPM());
  Serial.println("Avg BPM");
  Serial.println(particleSensor.getAvgBPM());

}
