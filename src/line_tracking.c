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

#define CENTER_TO_WALL 225

#define ROTATION_180 1
#define ROTATION_90 0

#define ROTATION_CLOCKWISE 1
#define ROTATION_ANTICLOCKWISE 0

static uint16_t string_coeff[] = {10, 13, 17, 22, 29, 33};
static uint16_t string_distance[] = {79, 133, 186, 240, 285, 335};

static uint8_t line_detected = 0;
static int16_t speed_correction = 0;

static WALL_FACED wall_faced = WALL_2;

// simple sign function: 1 if positive, -1 if negative, 0 if 0
int8_t sign(int16_t number)
{
    if (number > 0)
        return 1;
    else if (number < 0)
        return -1;
    else
        return 0;
}

// returns the wall currently faced
WALL_FACED get_wall_faced(void)
{
    return wall_faced;
}

void set_wall_faced(WALL_FACED new_wall_faced)
{
    wall_faced = new_wall_faced;
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

// tracks a line until condition = false. If direction = 1 it goes forward, if direction = -1 it goeas backwards
// when condition reached (condition = true) motors are stopped
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

// handle the rotation of the robot of 90 or 180, clockwise or anticlockwise depending of the previous state of the FSM
void rotation_robot(bool angle_degree, bool clockwise, FSM_STATE prev_state)
{
    static uint32_t steps_done = 0;
    if (angle_degree) // rotation 180°
    {
        if (steps_done >= STEPS_FOR_180_ROTATION)
        {
            right_motor_set_speed(0);
            left_motor_set_speed(0);
            steps_done = 0;
            // little optimisation using modulo: it changes wall faced between 1 and 3
            set_wall_faced((wall_faced + 2) % 4);

            switch (prev_state)
            {
            case FREQUENCY_POSITION:
                // chprintf((BaseSequentialStream *)&SD3, "FREQUENCY POSITION -r \n");
                // chThdSleepMilliseconds(100);
                increment_FSM_state();
                break;
            case STRING_CENTER:
                // chprintf((BaseSequentialStream *)&SD3, "STRING CENTER -r \n");
                // chThdSleepMilliseconds(100);
                set_FSM_state(STRING_CENTER);
                break;
            default:
                // chprintf((BaseSequentialStream *)&SD3, "DO NOTHING -r \n");
                // chThdSleepMilliseconds(100);
                set_FSM_state(DO_NOTHING);
            }
        }
        else
        {
            right_motor_set_speed(-MOTOR_SPEED_LIMIT);
            left_motor_set_speed(MOTOR_SPEED_LIMIT);
            steps_done++;
        }
    }
    else // rotation 90°
    {
        if (clockwise)
        {
            right_motor_set_speed(-MOTOR_SPEED_LIMIT);
            left_motor_set_speed(MOTOR_SPEED_LIMIT);
            steps_done++;
        }

        else
        {
            right_motor_set_speed(MOTOR_SPEED_LIMIT);
            left_motor_set_speed(-MOTOR_SPEED_LIMIT);
            steps_done++;
        }

        if (steps_done >= STEPS_FOR_90_ROTATION)
        {
            right_motor_set_speed(0);
            left_motor_set_speed(0);
            steps_done = 0;

            switch (prev_state)
            {
            case STRING_POSITION:
                select_color_detection(RED_COLOR);
                clockwise ? set_wall_faced(WALL_3) : set_wall_faced(WALL_1);
                chprintf((BaseSequentialStream *)&SD3, "FREQUENCY POSITION -r \n");
                // chThdSleepMilliseconds(100);
                increment_FSM_state();
                break;
            case STRING_CENTER:
                select_color_detection(BLUE_COLOR);
                set_wall_faced(WALL_2);
                // chprintf((BaseSequentialStream *)&SD3, "STRING POSITION -r \n");
                // chThdSleepMilliseconds(100);
                set_FSM_state(STRING_POSITION);
                break;
            default:
                // chprintf((BaseSequentialStream *)&SD3, "DO NOTHING -r \n");
                // chThdSleepMilliseconds(100);
                set_FSM_state(DO_NOTHING);
            }
        }
    }
}

static THD_WORKING_AREA(waPiRegulator, 512);
static THD_FUNCTION(PiRegulator, arg)
{

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    int16_t distance_diff = 0;
    FSM_STATE current_state = 0;
    FSM_STATE previous_state = 0;
    static bool distance_reached = false;

    while (1)
    {

        time = chVTGetSystemTime();
        current_state = get_FSM_state();
        previous_state = get_FSM_previous_state();
        distance_diff = 0;

        if (current_state == STRING_POSITION)
        {

            distance_reached = false;
            // difference between the goal distance and the measured distance
            distance_diff = VL53L0X_get_dist_mm() - string_distance[get_guitar_string() - 1];
            if (abs(distance_diff) <= TOF_PRECISION)
            {
                distance_reached = true;
                // chprintf((BaseSequentialStream *)&SD3, "String position reached \n");
                chprintf((BaseSequentialStream *)&SD3, "ROTATION -sp \n");
                // chThdSleepMilliseconds(100);
                increment_FSM_state();
            }
            line_tracking_while_condition(distance_reached, sign(distance_diff));
        }
        else if (current_state == ROTATION)
        {
            switch (previous_state)
            {
            case STRING_POSITION:
                // en fonction de la frequence > au centre ou pas
                rotation_robot(ROTATION_90, get_pitch(), previous_state);
                break;
            case FREQUENCY_POSITION:
                rotation_robot(ROTATION_180, ROTATION_CLOCKWISE, previous_state);
                break;
            case STRING_CENTER:
                if (distance_reached)
                    rotation_robot(ROTATION_90, wall_faced % 3, previous_state);
                else
                    rotation_robot(ROTATION_180, ROTATION_CLOCKWISE, previous_state);
                break;
            default:
                break;
            }
        }
        else if (current_state == FREQUENCY_POSITION)
        {
            distance_reached = false;
            // distance calculé par rapport au mur 1
            uint16_t distance_frequency = CENTER_TO_WALL + (get_frequency() - get_string_frequency()) * string_coeff[get_guitar_string() - 1];
            chprintf((BaseSequentialStream *)&SD3, "WAll faced = %d\n", wall_faced);
            if (wall_faced == 3)
            {
                // distance calculé par rapport au mur 3
                distance_frequency = CENTER_TO_WALL * 2 - distance_frequency;
            }
            chprintf((BaseSequentialStream *)&SD3, "distance frequency = %u\n", distance_frequency);
            chThdSleepMilliseconds(50);
            // difference between the goal distance and the measured distance
            distance_diff = VL53L0X_get_dist_mm() - distance_frequency;

            if (distance_diff >= TOF_PRECISION)
            {
                line_tracking_while_condition(distance_reached, 1);
            }
            else if (distance_diff <= -TOF_PRECISION)
            {
                // right_motor_set_speed(0);
                // left_motor_set_speed(0);
                chprintf((BaseSequentialStream *)&SD3, "ROTATION -fp \n");
                // chThdSleepMilliseconds(100);
                set_FSM_state(ROTATION);
            }
            else // the robot is in the good position
            {
                distance_reached = true;
                line_tracking_while_condition(distance_reached, 1);
                // chprintf((BaseSequentialStream *)&SD3, "Frequency position reached \n");
                // chprintf((BaseSequentialStream *)&SD3, "FREQUENCY DETECTION -fp \n");
                // chThdSleepMilliseconds(100);
                set_FSM_state(FREQUENCY_DETECTION);
            }
        }

        else if (current_state == STRING_CENTER)
        {
            distance_reached = false;
            // difference between the goal distance and the measured distance
            distance_diff = VL53L0X_get_dist_mm() - CENTER_TO_WALL;

            if (distance_diff >= TOF_PRECISION)
            {
                line_tracking_while_condition(distance_reached, 1);
            }
            else if (distance_diff <= -TOF_PRECISION)
            {
                // right_motor_set_speed(0);
                // left_motor_set_speed(0);
                // chprintf((BaseSequentialStream *)&SD3, "ROTATION -sc \n");
                // chThdSleepMilliseconds(100);
                set_FSM_state(ROTATION);
            }
            else
            {
                distance_reached = true;
                line_tracking_while_condition(distance_reached, 1);
                // chprintf((BaseSequentialStream *)&SD3, "String center reached /n");
                // chprintf((BaseSequentialStream *)&SD3, "ROTATION -sc \n");
                // chThdSleepMilliseconds(100);
                set_FSM_state(ROTATION);
            }
        }
        // 100Hz
        // chThdSleepUntilWindowed(time, time + MS2ST(10));
        chThdSleepMilliseconds(10);
    }
}

// static THD_WORKING_AREA(waRotation, 256);
// static THD_FUNCTION(Rotation, arg)
// {

//     chRegSetThreadName(__FUNCTION__);
//     (void)arg;

//     FSM_STATE current_state = 0;
//     FSM_STATE previous_state = 0;

//     while (1)
//     {
//         current_state = get_FSM_state();
//         previous_state = get_FSM_previous_state();

//         if (current_state == ROTATION)
//         {
//             switch (previous_state)
//             {
//             case STRING_POSITION:
//                 // en fonction de la frequence > au centre ou pas
//                 rotation_robot(ROTATION_90, get_pitch(), previous_state);
//                 break;
//             case FREQUENCY_POSITION:
//                 rotation_robot(ROTATION_180, ROTATION_CLOCKWISE, previous_state);
//                 break;
//             case STRING_CENTER:
//                 if (distance_reached)
//                     rotation_robot(ROTATION_90, wall_faced % 3, previous_state);
//                 else
//                     rotation_robot(ROTATION_180, ROTATION_CLOCKWISE, previous_state);
//                 break;
//             default:
//                 break;
//             }
//         }
//         // 100Hz
//         chThdSleepMilliseconds(100);
//     }
// }

void pi_regulator_start(void)
{
    chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
    // chThdCreateStatic(waRotation, sizeof(waRotation), NORMALPRIO, Rotation, NULL);
}
