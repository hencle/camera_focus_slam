// position io the linear position of the objective lens in mm
// pos_3d is the centeredn focus position of the objective lens in 3d space

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include <SPI.h>
#include <TMCStepper.h>
#include "AccelStepper.h"
#include "motion.h"
#include "start_seq.h"






// ########### interrupts ###########
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
void IRAM_ATTR handleInterrupt();







// ########### functions ###########
void print_motor_status();
void print_motor_status_ms();

void stall_detection();
void stall_detection_spi_reg();

void debugPrint(String str);


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

  pinMode(stall_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(stall_pin), handleInterrupt, HIGH);

  // Print a message to the serial port
  Serial.println("Stepper motor ready!");
delay(10);
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