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
#include "image_processing.h"
#include "main.h"

#define SPEED_MOTORS 200
#define TOF_PRECISION 5 // in [mm]
#define ERROR_THRESHOLD 10
#define MAX_ERROR 150
#define KP 0.5f // proportional constant

#define STEPS_FOR_90_ROTATION 30
#define STEPS_FOR_180_ROTATION 60
#define ROTATION_180 1
#define ROTATION_90 0
#define ROTATION_CLOCKWISE 1

// distance between the center line and the walls 1 and 3
#define CENTER_TO_WALL 225
// structure to communicate values to the RGB LEDs
// coefficients to convert the frequency difference into a distance, for each string
static uint16_t string_coeff[] = {10, 13, 17, 22, 29, 33};
// distance of each string from the WALL_2
static uint16_t string_distance[] = {79, 133, 186, 240, 285, 335};

static bool line_detected = 0;
static int16_t speed_correction = 0;
static WALL_FACED wall_faced = WALL_2;

/*
 *	Simple sign function that returns:   1 if the number is positive
 *                                      -1 if the number is negative
 *                                       0 if the number is zero
 *	params :
 *	int16_t number		                 Signed number.
 */
int8_t sign(int16_t number)
{
    if (number > 0)
        return 1;
    else if (number < 0)
        return -1;
    else
        return 0;
}

/*
 *	Returns the wall currently faced by the TOF of the EPuck2.
 */
WALL_FACED get_wall_faced(void)
{
    return wall_faced;
}

/*
 *	Set the new wall faced by the TOF of the EPuck2.
 *
 *  params:
 *  WALL_FACED new_wall_faced           New wall faced by the TOF.
 */
void set_wall_faced(WALL_FACED new_wall_faced)
{
    wall_faced = new_wall_faced;
}

/*
 *	Simple P regulator implementation: the speed is proportionnal
 *  to the error (distance - goal).
 *
 *  params:
 *  uint16_t distance           Current distance measured.
 *  uint16_t goal               Goal distance.
 */
int16_t p_regulator(uint16_t distance, uint16_t goal)
{

    int16_t error = 0;
    int16_t speed = 0;

    error = distance - goal;

    // disables the P regulator if the error is to small, the camera is a bit noisy
    if (abs(error) < ERROR_THRESHOLD)
    {
        return 0;
    }
    // we set a maximum and a minimum error
    else if (error > MAX_ERROR)
    {
        error = MAX_ERROR;
    }
    else if (error < -MAX_ERROR)
    {
        error = -MAX_ERROR;
    }

    speed = KP * error;

    return speed;
}

struct RGB color_led_distance(int16_t distance_center)
{
    int16_t distance_center_tpm = distance_center;
    distance_center_tpm = (abs(distance_center_tpm) > 195) ? 195 : distance_center_tpm;
    struct RGB param_rgb;
    param_rgb.b_value = 0;
    param_rgb.g_value = (uint8_t)(255 - (1.3 * abs(distance_center_tpm)));
    param_rgb.r_value = (uint8_t)(1.3 * abs(distance_center_tpm));

    return param_rgb;
}

/*
 *  Tracks and follow a line detected by the camera while the parameter "condition" is false.
 *  When the condition is reached (condition = true) motors are stopped.
 *
 *  params:
 *  bool condition                Condition to stop the EPuck2:  if false the EPuck2 tracks the line.
 *                                                               if true the EPuck2 stops.
 *  int8_t direction              Direction of the EPuck2:       1 is forward.
 *                                                              -1 is backward.
 */
void line_tracking_while_condition(bool condition, int8_t direction)
{
    if (!condition)
    {
        // search for a line
        line_detected = get_line_detection();

        if (line_detected)
        {
            // computes a correction factor to let the robot rotate to be in front of the line
            speed_correction = p_regulator(get_line_position(), (IMAGE_BUFFER_SIZE / 2));

            right_motor_set_speed(direction * SPEED_MOTORS - speed_correction);
            left_motor_set_speed(direction * SPEED_MOTORS + speed_correction);
        }
        // if no line found, motors are turned off
        else
        {
            right_motor_set_speed(0);
            left_motor_set_speed(0);
        }
    }
    else // stop the motors
    {
        right_motor_set_speed(0);
        left_motor_set_speed(0);
    }
}

/*
 *  Handle the rotation of the robot depending on the previous state of the FSM.
 *
 *  params:
 *  bool angle_degree               Angle of the rotation performed by the EPuck2: true is 180deg
 *                                                                                 false is 90deg
 *  bool clockwise                  Rotation direction: true is clocwise
 *                                                      false is anticlockwise
 *  FSM_STATE prev_state            Previous state of the FSM.
 */
void rotation_robot(bool angle_degree, bool clockwise, FSM_STATE prev_state)
{
    static uint8_t steps_done = 0;
    if (angle_degree) // rotation 180°
    {
        right_motor_set_speed(-MOTOR_SPEED_LIMIT);
        left_motor_set_speed(MOTOR_SPEED_LIMIT);
        steps_done++;

        if (steps_done >= STEPS_FOR_180_ROTATION)
        {
            right_motor_set_speed(0);
            left_motor_set_speed(0);
            steps_done = 0;
            // set the new wall faced: WALL_1 if previous was WALL_3, and viceversa
            set_wall_faced((wall_faced + 2) % 4);

            switch (prev_state)
            {
            case FREQUENCY_POSITION:
                // chprintf((BaseSequentialStream *)&SD3, "FREQUENCY POSITION -r \n");
                // chThdSleepMilliseconds(100);
                clear_rgb_leds();
                increment_FSM_state();
                break;
            case STRING_CENTER:
                // chprintf((BaseSequentialStream *)&SD3, "STRING CENTER -r \n");
                // chThdSleepMilliseconds(100);
                clear_rgb_leds();
                set_FSM_state(STRING_CENTER);
                break;
            default:
                clear_rgb_leds();
                set_FSM_state(DO_NOTHING);
            }
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
                set_FSM_state(DO_NOTHING);
            }
        }
    }
}

/*
 *  Thread that tracks and follow a line drawn on the ground,
 *  until a distance is reached, according to the current
 *  state of the FSM. The distance is calculated using
 *  the frequency detected.
 */
static THD_WORKING_AREA(waLineTracking, 512);
static THD_FUNCTION(LineTracking, arg)
{

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    // difference between the measured distance and the goal distance
    int16_t distance_diff = 0;
    FSM_STATE current_state = 0;
    FSM_STATE previous_state = 0;
    static bool distance_reached = false;

    while (1)
    {
        current_state = get_FSM_state();
        previous_state = get_FSM_previous_state();
        distance_diff = 0;

        if (current_state == STRING_POSITION)
        {
            set_all_rgb_leds(0, 255, 0);
            distance_reached = false;
            // difference between the measured distance and the distance of the current string
            distance_diff = VL53L0X_get_dist_mm() - string_distance[get_guitar_string() - 1];
            if (abs(distance_diff) <= TOF_PRECISION)
            {
                distance_reached = true;
                // chprintf((BaseSequentialStream *)&SD3, "String position reached \n");
                chprintf((BaseSequentialStream *)&SD3, "ROTATION -sp \n");
                clear_rgb_leds();
                increment_FSM_state();
            }
            // if the distance is not reached, follow the line
            line_tracking_while_condition(distance_reached, sign(distance_diff));
        }
        else if (current_state == ROTATION)
        {
            switch (previous_state)
            {
            case STRING_POSITION:
                set_all_rgb_leds(255, 255, 0);
                // rotation of 90deg, clocwise or anticlockwise depending on the pitch of the frequency
                rotation_robot(ROTATION_90, get_pitch(), previous_state);
                break;
            case FREQUENCY_POSITION:
                set_all_rgb_leds(255, 255, 0);
                // clocwise rotation of 180deg, the goal distance is behind the robot.
                rotation_robot(ROTATION_180, ROTATION_CLOCKWISE, previous_state);
                break;
            case STRING_CENTER:
                if (distance_reached)
                {
                    set_all_rgb_leds(0, 255, 0);
                    // the robot is at the center. Rotation of 90deg to face the WALL_2
                    rotation_robot(ROTATION_90, wall_faced % 3, previous_state);
                }
                else
                {
                    set_all_rgb_leds(0, 255, 0);
                    // clocwise rotation of 180deg, the goal distance is behind the robot.
                    rotation_robot(ROTATION_180, ROTATION_CLOCKWISE, previous_state);
                }
                break;
            default:
                clear_rgb_leds();
                set_FSM_state(DO_NOTHING);
                break;
            }
        }
        else if (current_state == FREQUENCY_POSITION)
        {
            distance_reached = false;
            // distance calculated from the WALL_1, using the frequency and the string detected
            uint16_t distance_frequency = CENTER_TO_WALL + (get_frequency() - get_string_frequency()) * string_coeff[get_guitar_string() - 1];
            chprintf((BaseSequentialStream *)&SD3, "WAll faced = %d\n", wall_faced);
            if (wall_faced == 3)
            {
                // distance converted. Now the distance is from the WALL_3
                distance_frequency = CENTER_TO_WALL * 2 - distance_frequency;
            }

            set_all_rgb_leds(color_led_distance(255 - distance_frequency).r_value, color_led_distance(255 - distance_frequency).g_value, 0);
            chprintf((BaseSequentialStream *)&SD3, "distance frequency = %u\n", distance_frequency);
            chThdSleepMilliseconds(50);
            // difference between the measured distance and the distance to the wall
            distance_diff = VL53L0X_get_dist_mm() - distance_frequency;

            if (distance_diff >= TOF_PRECISION)
            {
                // the goal distance is in front of the robot -> follow the line
                line_tracking_while_condition(distance_reached, 1);
            }
            else if (distance_diff <= -TOF_PRECISION)
            {
                // the goal distance is behind the robot -> 180deg rotation
                chprintf((BaseSequentialStream *)&SD3, "ROTATION -fp \n");
                clear_rgb_leds();
                set_FSM_state(ROTATION);
            }
            else // the robot reached his goal
            {
                distance_reached = true;
                // stop the robot
                line_tracking_while_condition(distance_reached, 1);
                // chprintf((BaseSequentialStream *)&SD3, "Frequency position reached \n");
                // chprintf((BaseSequentialStream *)&SD3, "FREQUENCY DETECTION -fp \n");
                // chThdSleepMilliseconds(100);
                // ready to detect a new frequency
                clear_rgb_leds();
                set_FSM_state(FREQUENCY_DETECTION);
            }
        }

        else if (current_state == STRING_CENTER)
        {
            set_all_rgb_leds(0, 255, 0);
            distance_reached = false;
            // difference between the measured distance and the center line
            distance_diff = VL53L0X_get_dist_mm() - CENTER_TO_WALL;

            if (distance_diff >= TOF_PRECISION)
            {
                // the goal distance is in front of the robot -> follow the line
                line_tracking_while_condition(distance_reached, 1);
            }
            else if (distance_diff <= -TOF_PRECISION)
            {
                // chprintf((BaseSequentialStream *)&SD3, "ROTATION -sc \n");
                // chThdSleepMilliseconds(100);
                // the goal distance is behind the robot -> 180deg rotation
                clear_rgb_leds();
                set_FSM_state(ROTATION);
            }
            else // the robot reached his goal
            {
                distance_reached = true;
                // stop the robot
                line_tracking_while_condition(distance_reached, 1);
                // chprintf((BaseSequentialStream *)&SD3, "String center reached /n");
                // chprintf((BaseSequentialStream *)&SD3, "ROTATION -sc \n");
                // chThdSleepMilliseconds(100);
                // 90deg rotation to face de WALL_2
                clear_rgb_leds();
                set_FSM_state(ROTATION);
            }
        }
        // 100Hz
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

/*
 *  Start the LineTracking thread.
 */
void line_tracking_start(void)
{
    chThdCreateStatic(waLineTracking, sizeof(waLineTracking), NORMALPRIO, LineTracking, NULL);
    // chThdCreateStatic(waRotation, sizeof(waRotation), NORMALPRIO, Rotation, NULL);
}
