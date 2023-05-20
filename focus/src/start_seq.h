#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include <SPI.h>
#include <TMCStepper.h>
#include "AccelStepper.h"


// Define Motor Driver Pins
#define R_SENSE 0.11   // R_Sense Resisitor Value
#define STALL_VALUE -60 // [-64..63]
#define stall_pin 4

#define EN_PIN 22   // Enable
#define DIR_PIN 27  // Direction
#define STEP_PIN 26 // Step
#define CS_PIN 5    // Chip select

// #define SW_MOSI          19 // Software Master Out Slave In
// #define SW_MISO          23 // Software Master In Slave Out
// #define SW_SCL           18 // Software Slave Clock
// #define SW_CS           5 // Software Slave CS




// ########### Motor Driver ###########
using namespace TMC2130_n;
TMC2130_n::DRV_STATUS_t drv_status{0};

TMC2130Stepper tmc = TMC2130Stepper(CS_PIN, R_SENSE); // Hardware SPI
TMC2130Stepper tmc2 = TMC2130Stepper(CS_PIN, R_SENSE); // Hardware SPI
//TMC2130Stepper tmc3 = TMC2130Stepper(CS_PIN, R_SENSE); // Hardware SP

AccelStepper Motor_1 = AccelStepper(Motor_1.DRIVER, STEP_PIN, DIR_PIN);
AccelStepper Motor_2 = AccelStepper(Motor_2.DRIVER, STEP_PIN, DIR_PIN);
//AccelStepper Motor_3 = AccelStepper(Motor_3.DRIVER, STEP_PIN, DIR_PIN);




void init_motor_driver()
{
  // Define the stepper motor parameters
  Serial.println("\n\nMotor Driver 1 Setup");
  Serial.println("=====================================");

  // Driver Settings
  const int32_t rms_current_M1 = 500;
  const int32_t rms_current_M2 = 500;
  const int32_t rms_current_M3 = 500;

  const int32_t microsteps_M1 = 16;
  const float steps_per_mm_M1 = 80; // 80 for testing

  const int32_t v_max = 3;
  const int32_t a_max = 1;
  const int32_t v_min = 3;
  const int32_t a_min = 1;

  pinMode(CS_PIN, OUTPUT);    // enabble TMC SPI configuration
  digitalWrite(CS_PIN, HIGH); // Chip Select

  tmc.begin();          // Initiate pins and registeries
  tmc.rms_current(600);
  tmc.en_pwm_mode(0);   // Enable extremely quiet stepping
  // tmc.pwm_autoscale(1);
  tmc.microsteps(16);
  tmc.toff(4);
  tmc.blank_time(24);
  // tmc.TCOOLTHRS(0xFFFFF); // 20bit max
  // tmc.THIGH(0);
  tmc.semin(5);
  tmc.semax(2);
  tmc.sedn(0b01);
  tmc.sgt(STALL_VALUE);
  tmc.sfilt(1);
  //tmc.diag1_stall(1);

  // Set Motion Control Parameter
  Motor_1.setMaxSpeed(v_max * steps_per_mm_M1);
  Motor_1.setAcceleration(a_max * steps_per_mm_M1);

  Motor_1.setEnablePin(EN_PIN);
  Motor_1.setPinsInverted(false, false, true);
  Motor_1.enableOutputs(); // Enables the motor outputs after setup


  delay(10);
  Serial.println("Stepper motor ready!");
}