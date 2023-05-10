#include <ch.h>
#include <hal.h>
#include <math.h>
#include <arm_math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <motors.h>
#include <sensors/VL53L0X/VL53L0X.h>

#include "audio_processing.h"
#include "line_tracking.h"
#include "process_image.h"
// include main to include stdlib.h for the abs() function
#include "main.h"

// tableau avec les frequences exactes de chaque corde de la guitare (en ordre: de la première corde à la sixième)
static float string_frequency[] = {FIRST_STRING_FREQ, SECOND_STRING_FREQ, THIRD_STRING_FREQ, FOURTH_STRING_FREQ, FIFTH_STRING_FREQ, SIXTH_STRING_FREQ};
static uint16_t string_coeff[] = {13, 17, 22, 29, 37, 43};
static uint16_t string_distance[] = {16, 99, 175, 245, 315, 385};

int8_t sign(int16_t number)
{
    if (number > 0)
        return 1;
    else if (number < 0)
        return -1;
    else
        return 0;
}

// simple PI regulator implementation
int16_t pi_regulator(float distance, float goal)
{
    float error = 0;
    float speed = 0;

    static float sum_error = 0;

    error = distance - goal;

    // disables the PI regulator if the error is to small
    // this avoids to always move as we cannot exactly be where we want and
    // the camera is a bit noisy

    // fabs() => returns the absolute value of a float number
    if (fabs(error) < ERROR_THRESHOLD)
    {
        return 0;
    }

    sum_error += error;

    // we set a maximum and a minimum for the sum to avoid an uncontrolled growth
    if (sum_error > MAX_SUM_ERROR)
    {
        sum_error = MAX_SUM_ERROR;
    }
    else if (sum_error < -MAX_SUM_ERROR)
    {
        sum_error = -MAX_SUM_ERROR;
    }

    speed = KP * error + KI * sum_error;

    return (int16_t)speed;
}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg)
{

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    static uint8_t line_detected = 0;

    FSM_STATE current_state = 0;
    bool distance_reached = true;
    int16_t distance_diff = 0;

    int16_t speed = 0;
    int16_t speed_correction = 0;

    while (1)
    {

        time = chVTGetSystemTime();
        current_state = get_FSM_state();
        distance_reached = true;
        distance_diff = 0;

        if (current_state == STRING_POSITION)
        {
            distance_reached = false;
            // difference between the goal distance and the measured distance
            distance_diff = VL53L0X_get_dist_mm() - string_distance[get_guitar_string() - 1];
            if (abs(distance_diff) <= TOF_PRECISION)
            {
                distance_reached = true;
                chprintf((BaseSequentialStream *)&SD3, "String position finished = %u\n", string_distance[get_guitar_string() - 1]);
                chThdSleepMilliseconds(500);
                select_color_detection(RED_COLOR);
                increment_FSM_state();
            }
        }
        if (current_state == FREQUENCY_POSITION)
        {

            distance_reached = false;
            GUITAR_STRING string_number = get_guitar_string();
            uint16_t distance_frequency = 300 + (string_frequency[string_number - 1] - get_frequency()) * string_coeff[string_number - 1];
            // difference between the goal distance and the measured distance
            distance_diff = VL53L0X_get_dist_mm() - distance_frequency;
            if (abs(distance_diff) <= TOF_PRECISION)
            {
                chprintf((BaseSequentialStream *)&SD3, "Frequency position finished = %d\n", distance_frequency);
                distance_reached = true;
                increment_FSM_state();
            }
        }
        if (!distance_reached)
        {

            // search for a line (0) not found, (1) found
            line_detected = get_line_detection();
            // computes the speed to give to the motors
            // distance_cm is modified by the image processing thread
            // speed = pi_regulator(get_distance_cm(), GOAL_DISTANCE);

            // motors enables and line found -> follow the line
            if (line_detected)
            {
                // computes a correction factor to let the robot rotate to be in front of the line
                speed_correction = (get_line_position() - (IMAGE_BUFFER_SIZE / 2));
                // SendUint16ToRealTerm(speed_correction + (IMAGE_BUFFER_SIZE / 2));

                // if the line is nearly in front of the camera, don't rotate
                if (abs(speed_correction) < ROTATION_THRESHOLD)
                {
                    speed_correction = 0;
                }

                // applies the speed from the PI regulator and the correction for the rotation
                // right_motor_set_speed(speed - ROTATION_COEFF * speed_correction);
                // left_motor_set_speed(speed + ROTATION_COEFF * speed_correction);

                right_motor_set_speed(sign(distance_diff) * (SPEED_MOTORS - ROTATION_COEFF * speed_correction));
                left_motor_set_speed(sign(distance_diff) * (SPEED_MOTORS + ROTATION_COEFF * speed_correction));
            }
            // motors enabled but no line found -> e-puck rotates until finds a line
            else
            {
                // // choose to rotate clockwise
                // #if ROTATION_CLOCKWISE
                //                 right_motor_set_speed(-100);
                //                 left_motor_set_speed(100);
                // #endif

                // // choose to rotate anticlockwise
                // #if !ROTATION_CLOCKWISE
                //                 right_motor_set_speed(100);
                //                 left_motor_set_speed(-100);

                // #endif
                right_motor_set_speed(0);
                left_motor_set_speed(0);
            }
        }
        else
        {
            right_motor_set_speed(0);
            left_motor_set_speed(0);
        }
        // 100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void)
{
    chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}

void robot_rotate(float pas, bool clockwise)
{
    float rotation_cm = (WHEELS_DISTANCE_CM / 2.0) * (2.0 * PI * pas / 360.0);
    int32_t steps_to_rotate = (int32_t)round(rotation_cm * NSTEP_ONE_TURN / WHEEL_PERIMETER);

    // int32_t time_to_rotate_us = (int32_t)round((float)steps_to_rotate / SPEED_STEPS_PER_SECOND * TIME_CONVERSION_FACTOR);
    static u_int32_t step_counter = 0;
    while (step_counter < steps_to_rotate)
    {

        if (clockwise)
        {
            left_motor_set_speed(SPEED_MOTORS);
            right_motor_set_speed(-SPEED_MOTORS);
        }
        else
        {
            left_motor_set_speed(-SPEED_MOTORS);
            right_motor_set_speed(SPEED_MOTORS);
        }
        chThdSleepMilliseconds(10);
        step_counter++;
    }

    left_motor_set_speed(0);
    right_motor_set_speed(0);
    chprintf((BaseSequentialStream *)&SD3, "step counter finished = %d\n", step_counter);
    step_counter = 0;

    chprintf((BaseSequentialStream *)&SD3, "Rotation FINISHED = \n");
    increment_FSM_state();
}
