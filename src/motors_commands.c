#include "main.h"
#include "audio_processing.h"
#include "communications.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ch.h>
#include <hal.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <motors.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <leds.h>

void stop_when_string_found(void)
{
    uint16_t distance = VL53L0X_get_dist_mm();
    uint16_t string_distance = get_guitar_string() * DISTANCE_STRING;
    // check if the distance between the robot and the wall is equal to string_distance +- TOF_PRECISION
    if (string_distance - TOF_PRECISION < distance && distance < string_distance + TOF_PRECISION)
    {
        left_motor_set_speed(0);
        right_motor_set_speed(0);
        increment_FSM_state();
    }
    else
    {
        left_motor_set_speed(SPEED_MOTORS);
        right_motor_set_speed(SPEED_MOTORS);
    }
}