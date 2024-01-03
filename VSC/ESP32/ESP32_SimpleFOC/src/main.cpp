#include <Arduino.h>
#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include <Wire.h>
#include <SPI.h>
#include "encoders/as5048a/MagneticSensorAS5048A.h"
#include "driver/mcpwm.h"
#include "stdint.h"

//#define SPI_ENCODER
//#define I2C_ENCODER
#define PWM_ENCODER
#define TEST_PWM_ENCODER

#define VOLTAGE_12V 12


#ifdef SPI_ENCODER
#define SENSOR1_CS 15 // I_0
MagneticSensorAS5048A sensor1(SENSOR1_CS);
#endif

#ifdef I2C_ENCODER

MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor2 = MagneticSensorI2C(AS5600_I2C);

TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);
#endif

#ifdef PWM_ENCODER
#define ENCODER_PWM_PIN_1 15
#define ENCODER_PWM_PIN_2 13

MagneticSensorPWM sensor1 = MagneticSensorPWM(ENCODER_PWM_PIN_1, 4, 904);
MagneticSensorPWM sensor2 = MagneticSensorPWM(ENCODER_PWM_PIN_2, 4, 904);

void doPWM1(){sensor1.handlePWM();}
void doPWM2(){sensor2.handlePWM();}
#endif

BLDCMotor motor1 = BLDCMotor(7);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(32, 33, 25, 22);

BLDCMotor motor2 = BLDCMotor(7);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(26, 27, 14, 12);

float target_velocity = 0;
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }


void setup() {
  Serial.begin(115200);

  
  

#ifdef SPI_ENCODER
  sensor1.init();
#endif

#ifdef PWM_ENCODER
  sensor1.init();
  sensor2.init();

  sensor1.enableInterrupt(doPWM1);
  sensor2.enableInterrupt(doPWM2);
#endif

#ifdef I2C_ENCODER
  I2Cone.begin(19,18, 400000); 
  I2Ctwo.begin(23,5, 400000);
  sensor1.init(&I2Cone);
  sensor2.init(&I2Ctwo);
#endif

#ifdef TEST_PWM_ENCODER
  int max_pulse= 0;
  int min_pulse = 10000;

  //Test SensorPWM Value
  for (int i = 0; i < 5; i++)
  {
    sensor1.update();

    // keep track of min and max
    if(sensor1.pulse_length_us > max_pulse) max_pulse = sensor1.pulse_length_us;
    else if(sensor1.pulse_length_us < min_pulse) min_pulse = sensor1.pulse_length_us;

    // display the raw count, and max and min raw count
    Serial.print("angle:");
    Serial.print(sensor1.getAngle());
    Serial.print("\t, raw:");
    Serial.print(sensor1.pulse_length_us);
    Serial.print("\t, min:");
    Serial.print(min_pulse);
    Serial.print("\t, max:");
    Serial.println(max_pulse);      
  }

#endif

  motor1.linkSensor(&sensor1);
  motor2.linkSensor(&sensor2);

  driver1.voltage_power_supply = VOLTAGE_12V;
  driver1.init();

  driver2.voltage_power_supply = VOLTAGE_12V;
  driver2.init();

  motor1.linkDriver(&driver1);
  motor2.linkDriver(&driver2);

  motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor2.foc_modulation = FOCModulationType::SpaceVectorPWM;

  motor1.controller = MotionControlType::angle;
  motor2.controller = MotionControlType::angle;

  //Speed PID Setting                                     
  motor1.PID_velocity.P = 0.1;             //According to the selected motor, modify the PID parameters here to achieve better results
  motor2.PID_velocity.P = 0.1;
  motor1.PID_velocity.I = 1;
  motor2.PID_velocity.I = 1;
  //Angle PID Setting 
  motor1.P_angle.P = 20;
  motor2.P_angle.P = 20;
  //Motor Maximum Limit Voltage
  motor1.voltage_limit = 1;                //According to the supply voltage, modify the value of voltage_limit here
  motor2.voltage_limit = 1;               //Also modify the value of voltage_limit here
  
  //Speed Low-pass Filter Time Constant
  motor1.LPF_velocity.Tf = 0.01;
  motor2.LPF_velocity.Tf = 0.01;

  //Maximum Velocity Limit Setting
  motor1.velocity_limit = 20;
  motor2.velocity_limit = 20;

  motor1.useMonitoring(Serial);
  motor2.useMonitoring(Serial);

  motor1.init();
  motor1.initFOC();

  motor2.init();
  motor2.initFOC();

  command.add('T', doTarget, "target velocity");
  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using serial terminal:"));
}

void loop() {
  Serial.print(sensor1.getAngle()); 
  Serial.print(" - "); 
  Serial.print(sensor2.getAngle());
  Serial.println();
  motor1.loopFOC();
  motor2.loopFOC();

  motor1.move(target_velocity);
  motor2.move(target_velocity);
  
  command.run();
}

