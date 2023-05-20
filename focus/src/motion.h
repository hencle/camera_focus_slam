#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include <SPI.h>
#include <TMCStepper.h>
#include "AccelStepper.h"

volatile bool stall_flag = false;
bool stall_flag_reg = false;

long int motor_pos_to_step(float angle, int motor_id){

    float ratio_egree_to_step_m1 = 80;
    float ratio_egree_to_step_m2 = 80;
    float ratio_egree_to_step_m3 = 1;

    if (motor_id == 1)
    {
        return angle * ratio_egree_to_step_m1;
    }
    else if (motor_id == 2)
    {
        return angle * ratio_egree_to_step_m2;
    }
    else if (motor_id == 3)
    {
        return angle * ratio_egree_to_step_m3;
    }
    
    
}


float chk_limit(float position, int motor_id) {
    // Define the limits for each motor
    float m1_min = 0;
    float m1_max = 100;
    float m2_min = 0;
    float m2_max = 100;
    float mfocus_min = 0;
    float mfocus_max = 100;

    int errorCounter = 0;

    if (motor_id == 1) {
        if (position < m1_min) {
            position = m1_min;
            if (errorCounter >= 1 && errorCounter <= 5) {
                Serial.println("m1_min " + String(position) + "°");
            }
            errorCounter++;
        } else if (position > m1_max) {
            position = m1_max;
            if (errorCounter >= 1 && errorCounter <= 5) {
                Serial.println("m1_max " + String(position) + "°");
            }
            errorCounter++;
        }
    } else if (motor_id == 2) {
        if (position < m2_min) {
            position = m2_min;
            if (errorCounter >= 1 && errorCounter <= 5) {
                Serial.println("m2_min " + String(position) + "°");
            }
            errorCounter++;
        } else if (position > m2_max) {
            position = m2_max;
            if (errorCounter >= 1 && errorCounter <= 5) {
                Serial.println("m2_max " + String(position) + "°");
            }
            errorCounter++;
        }
    } else if (motor_id == 3) {
        if (position < mfocus_min) {
            position = mfocus_min;
            if (errorCounter >= 1 && errorCounter <= 5) {
                Serial.println("mfocus_min " + String(position) + "mm");
            }
            errorCounter++;
        } else if (position > mfocus_max) {
            position = mfocus_max;
            if (errorCounter >= 1 && errorCounter <= 5) {
                Serial.println("mfocus_max " + String(position) + "mm");
            }
            errorCounter++;
        }
    } else {
        Serial.println("Error: motor_id not defined");
        if (errorCounter >= 1 && errorCounter <= 5) {
            // Perform actions based on the error activation
            // For example, stop other operations or flag an error state
        }
        errorCounter++;
    }

    return position;
}


