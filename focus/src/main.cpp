// position io the linear position of the objective lens in mm
// pos_3d is the centeredn focus position of the objective lens in 3d space

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

using namespace TMC2130_n;
TMC2130_n::DRV_STATUS_t drv_status{0};

TMC2130Stepper tmc = TMC2130Stepper(CS_PIN, R_SENSE); // Hardware SPI

AccelStepper Motor_1 = AccelStepper(Motor_1.DRIVER, STEP_PIN, DIR_PIN);
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
void IRAM_ATTR handleInterrupt();

void init_motor_driver();
void print_motor_status();
void print_motor_status_ms();

void stall_detection();
void stall_detection_spi_reg();

void debugPrint(String str);

volatile bool stall_flag = false;

bool stall_flag_reg = false;
double target_pos = 0;
bool target_reached = false;
unsigned long currentTime = millis();
unsigned long previousMillis = millis();
int printCount = 0;

// ##################
//       Setup
// ##################
void setup()
{
  // Initialize the serial port
  Serial.begin(115200);
  while (!Serial);

  SPI.begin();

  // Initialize the stepper motor
  init_motor_driver();

  // Print a message to the serial port
  Serial.println("Stepper motor ready!");
}


// ##################
//       loop
// ##################
void loop()
{
  currentTime = millis();
  // Check if there is any data available on the serial port
  if (Serial.available())
  {
    delay(500);
    // Read the data from the serial port
    char command = Serial.read();

    // If the command is "pos", move the stepper motor to the specified position
    if (command == 'p')
    {
      // Read the position from the serial port
      double position = Serial.parseFloat();
      target_pos = position;
      double distance = position - Motor_1.currentPosition();

      // Move the stepper motor to the specified position
      Motor_1.moveTo(position);
      Motor_1.enableOutputs();
      target_reached = false;
    }

    // If the command is "rel_pos", move the stepper motor to the specified relative position
    else if (command == 'r')
    {
      double position = Serial.parseFloat();

      Motor_1.move(position);
    }
  }

  // Update the stepper motor
  if (Motor_1.distanceToGo() != 0)
  {
    Motor_1.run();
    // Motor_2.run();
    // Motor_3.run();
  }
  else if (Motor_1.distanceToGo() == 0)
  {
    if (target_reached == false)
    {
      print_motor_status();
      // Motor_1.disableOutputs();
      // digitalWrite(EN_PIN, HIGH);
      target_reached = true;
    }
  }

  // Serial.println(Motor_1.distanceToGo());
  print_motor_status_ms();

  static uint32_t last_time = 0;
  uint32_t ms = millis();

  if ((ms - last_time) > 100)
  { 
    last_time = ms;

    stall_detection();
    stall_detection_spi_reg();
  }
}













// ###############################################
// ################## functions ##################
// ###############################################



// ############## interrupts ################
void IRAM_ATTR handleInterrupt()
{
  portENTER_CRITICAL_ISR(&mux);
  stall_flag = true;
  portEXIT_CRITICAL_ISR(&mux);
}

// ############### stall detection ################
void stall_detection()
{
  if (stall_flag == true)
  {
    Motor_1.stop();
    Motor_1.setCurrentPosition(0);
    //digitalWrite(EN_PIN, HIGH);

    portENTER_CRITICAL(&mux);
    stall_flag = false;
    portEXIT_CRITICAL(&mux);

    Serial.println("Stall detected by PIN interrupt!");
  }
}
// ############### stall detection spi ################
void stall_detection_spi_reg()
{

  stall_flag_reg = tmc.stallguard();



    // TMC5160 uses the same DRV_STATUS layout as 2130
    TMC2130_n::DRV_STATUS_t data{tmc.DRV_STATUS()};
    Serial.print("  \nstallguard: ");
    int val = map(data.sg_result, 0, 1024, 0, 100);
    Serial.print(val);
    Serial.print(" ");
    // if (++printCount >= 30) {
    //   printCount = 0;
    //   Serial.println();
    // }

  if (stall_flag_reg == true)
  {
    // Motor_1.stop();
    // Motor_1.setCurrentPosition(0);
    // digitalWrite(EN_PIN, HIGH);

    stall_flag_reg = false;

    //Serial.print("  SPI Stall detected ");
  }
}













// ###################################################
// ############### print motor status ################
// ###################################################
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

  pinMode(stall_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(stall_pin), handleInterrupt, HIGH);

  delay(10);
  Serial.println("Stepper motor ready!");
}

// ################## motor status ##################
void print_motor_status()
{
  Serial.println("Target pos: " + String(target_pos) + "  current pos: " + String(Motor_1.currentPosition()) + "  current speed: " + String(Motor_1.speed()) + " mm/s " + "  DistanceToGo: " + String(Motor_1.distanceToGo()) + " mm ");
}

void print_motor_status_ms()
{

  if (currentTime - previousMillis >= 5000)
  {
    Serial.println("Target pos: " + String(target_pos) + "  current pos: " + String(Motor_1.currentPosition()) + "  current speed: " + String(Motor_1.speed()) + " mm/s " + "  DistanceToGo: " + String(Motor_1.distanceToGo()) + " mm ");
    previousMillis = currentTime;
  }
}



void debugPrint(String str) {
  Serial.println(str);
  //delay(1000);
}