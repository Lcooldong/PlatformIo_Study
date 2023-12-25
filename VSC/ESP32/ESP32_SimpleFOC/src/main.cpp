#include <Arduino.h>
#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include <Wire.h>
#include <SPI.h>
#include "encoders/as5048a/MagneticSensorAS5048A.h"

//#define SPI_ENCODER


#ifdef SPI_ENCODER
#define SENSOR1_CS 15 // I_0
MagneticSensorAS5048A sensor1(SENSOR1_CS);
#endif

BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 22);



void setup() {
  Serial.begin(115200);

#ifdef SPI_ENCODER
  sensor1.init();
#endif

  motor.init();
  motor.initFOC();

}

void loop() {
 
}

