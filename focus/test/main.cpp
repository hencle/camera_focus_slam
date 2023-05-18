
#include <TMCStepper.h>
#include <AccelStepper.h>



#define STALL_VALUE 15 // [-64..63]
const byte stall_pin = 4;

#define EN_PIN 22   // Enable
#define DIR_PIN 27  // Direction
#define STEP_PIN 26 // Step
#define CS_PIN 5    // Chip select

#define R_SENSE 0.11f // Match to your driver
                      // SilentStepStick series use 0.11
                      // UltiMachine Einsy and Archim2 boards use 0.2
                      // Panucatt BSD2660 uses 0.1
                      // Watterott TMC5160 uses 0.075

// Select your stepper driver type
TMC2130Stepper driver(CS_PIN, R_SENSE);                           // Hardware SPI
AccelStepper stepper = AccelStepper(stepper.DRIVER, STEP_PIN, DIR_PIN);

using namespace TMC2130_n;

// Using direct register manipulation can reach faster stepping times
#define STEP_PORT     PORTF // Match with STEP_PIN
#define STEP_BIT_POS      0 // Match with STEP_PIN


#define MAX_SPEED        40 // In timer value
#define MIN_SPEED      1000




void setup() {
  SPI.begin();
  Serial.begin(115200);         // Init serial port and set baudrate
  while(!Serial);               // Wait for serial port to connect
  Serial.println("\nStart...");

  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);

  driver.begin();
    driver.rms_current(600); // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
  //driver.en_pwm_mode(1);   // Enable extremely quiet stepping
  //driver.pwm_autoscale(1);
  driver.microsteps(16);

  driver.toff(4);
  driver.blank_time(24);
  driver.TCOOLTHRS(0xFFFFF); // 20bit max
  driver.THIGH(0);
  driver.semin(5);
  driver.semax(2);
  driver.sedn(0b01);
  driver.sgt(STALL_VALUE);

    driver.diag1_stall(1);

  //driver.diag1_stall(1);
//   driver.toff(4);
//   driver.blank_time(24);
//   driver.rms_current(400); // mA
//   driver.microsteps(16);
//   driver.TCOOLTHRS(0xFFFFF); // 20bit max
//   driver.THIGH(0);
//   driver.semin(5);
//   driver.semax(2);
//   driver.sedn(0b01);
//   driver.sgt(STALL_VALUE);

}

void loop() {
  static uint32_t last_time=0;
  uint32_t ms = millis();



  if((ms-last_time) > 100) { //run every 0.1s
    last_time = ms;

    DRV_STATUS_t drv_status{0};
    drv_status.sr = driver.DRV_STATUS();

    Serial.print("0 ");
    Serial.print(drv_status.sg_result, DEC);
    Serial.print(" ");
    Serial.println(driver.cs2rms(drv_status.cs_actual), DEC);
  }
}
