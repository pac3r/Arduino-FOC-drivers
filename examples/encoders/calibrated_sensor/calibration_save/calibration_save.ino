/**
 * The example demonstrates the calibration of the magnetic sensor with the calibration procedure and saving the calibration data. 
 * So that the calibration procedure does not have to be run every time the motor is powered up.
 */

#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "encoders/calibrated/CalibratedSensor.h"

// fill this array with the calibration values outputed by the calibration procedure
float calibrationLut[50] = {0};
float zero_electric_angle = 0;
Direction sensor_direction = Direction::CW;

// magnetic sensor instance - SPI
MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, 14);
// Stepper motor & driver instance
StepperMotor motor = StepperMotor(50);
StepperDriver4PWM driver = StepperDriver4PWM(10, 9, 5, 6,8);
// instantiate the calibrated sensor object
// instantiate the calibrated sensor object
// argument 1 - sensor object
// argument 2 - number of samples in the LUT (default 200)
// argument 3 - pointer to the LUT array (defualt nullptr)
CalibratedSensor sensor_calibrated = CalibratedSensor(sensor, 50);

// voltage set point variable
float target_voltage = 2;

// instantiate the commander
Commander command = Commander(Serial);

void doTarget(char* cmd) { command.scalar(&target_voltage, cmd); }

void setup() {

  sensor.init();
  // Link motor to sensor
  motor.linkSensor(&sensor);
  // power supply voltage
  driver.voltage_power_supply = 20;
  driver.init();
  motor.linkDriver(&driver);
  // aligning voltage 
  motor.voltage_sensor_align = 8;
  motor.voltage_limit = 20;
  // set motion control loop to be used
  motor.controller = MotionControlType::torque;

  // use monitoring with serial 
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);
  motor.monitor_variables =  _MON_VEL; 
  motor.monitor_downsample = 10; // default 10

  // initialize motor
  motor.init();

  // Running calibration
  // the function will setup everything for the provided calibration LUT
  sensor_calibrated.calibrate(motor, calibrationLut, zero_electric_angle, sensor_direction);

  // Linking sensor to motor object
  motor.linkSensor(&sensor_calibrated);

  // calibrated init FOC
  motor.initFOC();

  // add target command T
  command.add('T', doTarget, "target voltage");
  
  Serial.println(F("Motor ready."));

  Serial.println(F("Set the target voltage using serial terminal:"));
  _delay(1000);
}

void loop() {

  motor.loopFOC();
  motor.move(target_voltage);
  command.run();
  motor.monitor();
 
}