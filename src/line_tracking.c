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
static uint16_t string_distance[] = {16, 105, 175, 245, 315, 385};

static uint8_t line_detected = 0;
static int16_t speed_correction = 0;
// 1 = clockwise rotation, -1 = anticlockwise rotation

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

void line_tracking_while_condition(bool condition, int8_t direction)
{
    if (!condition)
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

            right_motor_set_speed(direction * SPEED_MOTORS - ROTATION_COEFF * speed_correction);
            left_motor_set_speed(direction * SPEED_MOTORS + ROTATION_COEFF * speed_correction);
        }
        // no line found -> motors are turned off
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
}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg)
{

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    uint32_t counter_rotation = 0;
    int8_t clockwise_rotation = -1;
    int16_t distance_diff = 0;
    FSM_STATE current_state = 0;
    bool distance_reached = true;

    while (1)
    {

        time = chVTGetSystemTime();
        current_state = get_FSM_state();
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
            line_tracking_while_condition(distance_reached, sign(distance_diff));
        }
        else if (current_state == ROTATION)
        {
            FSM_STATE previous_state = get_FSM_previous_state();
            if (previous_state == STRING_POSITION)
            {
                clockwise_rotation = -1;
            }
            else
            {
                clockwise_rotation = 1;
            }

            if (counter_rotation >= 350)
            {
                right_motor_set_speed(0);
                left_motor_set_speed(0);
                counter_rotation = 0;
                if (previous_state == STRING_POSITION)
                {
                    increment_FSM_state();
                }
                else
                {
                    set_FSM_state(STRING_CENTER);
                }
            }
            else
            {
                right_motor_set_speed(-clockwise_rotation * 100);
                left_motor_set_speed(clockwise_rotation * 100);
                counter_rotation++;
            }
        }
        else if (current_state == FREQUENCY_POSITION)
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
                set_FSM_state(FREQUENCY_DETECTION);
            }
            line_tracking_while_condition(distance_reached, sign(distance_diff));
        }
        // 100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void)
{
    chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
