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

/*
 *   compares the real time distance of the epuck using the TOF with the distance_to_reach with a precision
 *   if the distance to reach is behind the epuck -> go backwards
 *   the distance to reach is in front of the epuck -> go ahead
 *   if the distance between the robot and the wall is equal to distance_to_reach +- precision -> Stop the epuck and increment the FSM
 *   distance and precisoin in mm
 */
void advance_until_distance_reached(uint16_t distance_to_reach, uint8_t precision)
{
    uint16_t distance = VL53L0X_get_dist_mm();
    // check if the distance between the robot and the wall is equal to distance_to_reach +- precision. Stop the epuck and increment the FSM
    if (distance_to_reach - precision < distance && distance < distance_to_reach + precision)
    {
        left_motor_set_speed(0);
        right_motor_set_speed(0);
        increment_FSM_state();
    }
    // if the distance to reach is behind the epuck -> go backwards
    else if (distance >= distance_to_reach + precision)
    {
        left_motor_set_speed(-SPEED_MOTORS);
        right_motor_set_speed(-SPEED_MOTORS);
    }
    // the distance to reach is in front of the epuck -> go ahead
    else
    {
        left_motor_set_speed(SPEED_MOTORS);
        right_motor_set_speed(SPEED_MOTORS);
    }
}