#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <sensors/VL53L0X/VL53L0X.h>

#include "ch.h"
#include "motors.h"
#include "main.h"
#include "leds.h"
#include "perpendicular_calibration.h"

#define FIRST_STEP 0
#define SECOND_STEP 1
#define CALIBRATION_IN_PROGRESS 2
#define CALIBRATION_LAST_PHASE 3
#define PAS 10.0
#define DISTANCE_START_TO_WALL 10.0
#define STEPS_FOR_TEN_DEGREES 47

#define NSTEP_ONE_TURN 1000  // number of step for 1 turn of the motor
#define WHEEL_PERIMETER 13   // [cm]
#define WHEELS_DISTANCE_CM 7 // [cm]

#define SPEED_STEPS_PER_SECOND 600
#define TIME_CONVERSION_FACTOR 1000000

static uint8_t calib_step = FIRST_STEP;
static float dist_0 = 0;
static float dist_n = 0;
static float dist_n_1 = 0;
static bool direction = 1;
static bool calibrating = true;

bool calibration(float dist)
{
    if (dist != 0)
    { // securité car la première valeur que me renvoie le tof est toujours 0
        if (calib_step == FIRST_STEP)
        {
            dist_0 = dist;
            robot_rotate(15.0, direction);
            chThdSleepMilliseconds(500);
            calib_step = SECOND_STEP;
        }
        else if (calib_step == SECOND_STEP)
        {
            dist_n = dist;
            if (dist_0 - dist_n > 0)
            {
                direction = 1;
                calib_step = CALIBRATION_IN_PROGRESS;
                robot_rotate(PAS, direction);
                chThdSleepMilliseconds(500);
            }
            else
            {
                direction = 0;
                calib_step = CALIBRATION_IN_PROGRESS;
                robot_rotate(PAS, direction);
                chThdSleepMilliseconds(500);
            }
        }
        else if (calib_step == CALIBRATION_IN_PROGRESS)
        {
            dist_n_1 = dist_n;
            dist_n = dist;
            if (dist_n >= dist_n_1)
            {
                calib_step = CALIBRATION_LAST_PHASE;
            }
            else
            {
                robot_rotate(PAS, direction);
                chThdSleepMilliseconds(500);
            }
        }
        else if (calib_step == CALIBRATION_LAST_PHASE)
        {
            dist_n = dist;
            float last_rotation_rad = acos(DISTANCE_START_TO_WALL / dist_n);
            robot_rotate(last_rotation_rad, !direction);
            chThdSleepMilliseconds(500);
            calib_step = FIRST_STEP;
            return false;
        }
    }
    return true;
}

void robot_rotate(float pas, bool clockwise)
{
    float rotation_cm = (WHEELS_DISTANCE_CM / 2.0) * (2.0 * M_PI * pas / 360.0);
    int32_t steps_to_rotate = (int32_t)round(rotation_cm * NSTEP_ONE_TURN / WHEEL_PERIMETER);

    int32_t time_to_rotate_us = (int32_t)round((float)steps_to_rotate / SPEED_STEPS_PER_SECOND * TIME_CONVERSION_FACTOR);

    if (!clockwise)
    {
        left_motor_set_speed(SPEED_STEPS_PER_SECOND);
        right_motor_set_speed(-SPEED_STEPS_PER_SECOND);
    }
    else
    {
        left_motor_set_speed(-SPEED_STEPS_PER_SECOND);
        right_motor_set_speed(SPEED_STEPS_PER_SECOND);
    }

    chThdSleepMicroseconds(time_to_rotate_us);

    left_motor_set_speed(0);
    right_motor_set_speed(0);
}

static THD_WORKING_AREA(waPerpendicularCalibration, 128);
static THD_FUNCTION(PerpendicularCalibration, arg)
{

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while (1)
    {
        // checks if the Finite State Machine is in the correct state
        if (get_FSM_state() == PERPENDICULAR_CALIBRATION)
        {
            if (calibrating)
            {
                set_front_led(1);
                chThdSleepMilliseconds(500);
                calibrating = calibration(VL53L0X_get_dist_mm());
                set_front_led(0);
                chThdSleepMilliseconds(500);
            }
            else
            {
                set_body_led(1);
                increment_FSM_state();
                chThdSleepMilliseconds(1000);
            }
        }
        else
        {
            chThdSleepMilliseconds(200);
        }
    }
}

void perpendicular_calibration_start(void)
{
    chThdCreateStatic(waPerpendicularCalibration, sizeof(waPerpendicularCalibration), NORMALPRIO, PerpendicularCalibration, NULL);
}