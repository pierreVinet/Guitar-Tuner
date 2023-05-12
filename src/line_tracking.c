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
#define WALL_POLARITY 2 - current_wall_faced

// tableau avec les frequences exactes de chaque corde de la guitare (en ordre: de la première corde à la sixième)
static float string_frequency[] = {FIRST_STRING_FREQ, SECOND_STRING_FREQ, THIRD_STRING_FREQ, FOURTH_STRING_FREQ, FIFTH_STRING_FREQ, SIXTH_STRING_FREQ};
static uint16_t string_coeff[] = {10, 13, 17, 22, 29, 33};
static uint16_t string_distance[] = {79, 133, 186, 240, 285, 335};

static uint8_t line_detected = 0;
static int16_t speed_correction = 0;

static uint32_t steps_done = 0;
static uint32_t steps_to_do = 0;

static WALL_FACED wall_faced = 0;

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

WALL_FACED get_wall_faced(void)
{
    return wall_faced;
}

void set_wall_faced(WALL_FACED new_wall_faced)
{
    wall_faced = new_wall_faced;
}

bool get_pitch(void)
{
    if (get_frequency() - string_frequency[get_guitar_string() - 1] > 0)
    {
        return 0;
    }
    else
    {
        return 1;
    }
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

void line_tracking_while_condition(bool condition)
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

            right_motor_set_speed(SPEED_MOTORS - ROTATION_COEFF * speed_correction);
            left_motor_set_speed(SPEED_MOTORS + ROTATION_COEFF * speed_correction);
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

void rotation_robot(bool angle_degree, bool clockwise)
{

    if (angle_degree) // rotation 180 °
    {
        if (steps_done >= STEPS_FOR_180_ROTATION)
        {
            right_motor_set_speed(0);
            left_motor_set_speed(0);
            steps_to_do = 0;
            increment_FSM_state();
            set_wall_faced((get_wall_faced() + 2) % 4);
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
        if (steps_done >= STEPS_FOR_90_ROTATION)
        {
            right_motor_set_speed(0);
            left_motor_set_speed(0);
            steps_to_do = 0;
            clockwise ? set_wall_faced(WALL_3) : set_wall_faced(WALL_1);

            switch (get_FSM_previous_state())
            {
            case STRING_POSITION:
                select_color_detection(RED_COLOR);
                increment_FSM_state();
                break;
            case FREQUENCY_POSITION:
                set_wall_faced(WALL_3);
                set_FSM_state(STRING_POSITION);
                break;
            default:
                select_color_detection(BLUE_COLOR);
                set_FSM_state(STRING_POSITION);
            }
        }
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
    }
}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg)
{

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    int16_t distance_diff = 0;
    FSM_STATE current_state = 0;
    WALL_FACED current_wall_faced = 0;
    bool distance_reached = true;

    while (1)
    {

        time = chVTGetSystemTime();
        current_state = get_FSM_state();
        current_wall_faced = get_wall_faced();
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
                increment_FSM_state();
            }

            line_tracking_while_condition(distance_reached);
        }
        else if (current_state == ROTATION)
        {
            FSM_STATE previous_state = get_FSM_previous_state();
            switch (previous_state)
            {
            case STRING_POSITION:
                // en fonction de la frequence > au centre ou pas
                rotation_robot(1, get_pitch());
                break;
            case FREQUENCY_POSITION:
                rotation_robot(0, 1);
                break;
            case STRING_CENTER:
                rotation_robot(1, get_wall_faced() % 3);
                break;

            default:
                break;
            }
        }
        else if (current_state == FREQUENCY_POSITION)
        {

            distance_reached = false;
            GUITAR_STRING string_number = get_guitar_string();

            uint16_t distance_frequency = CENTER_TO_WALL + (WALL_POLARITY) * (string_frequency[string_number - 1] - get_frequency()) * string_coeff[string_number - 1];
            // difference between the goal distance and the measured distance
            distance_diff = VL53L0X_get_dist_mm() - distance_frequency;

            if (distance_diff > 0)
            {

                if (abs(distance_diff) <= TOF_PRECISION)
                {
                    chprintf((BaseSequentialStream *)&SD3, "Frequency position finished = %d\n", distance_frequency);
                    distance_reached = true;
                    set_FSM_state(FREQUENCY_DETECTION);
                }
                line_tracking_while_condition(distance_reached);
            }
            else
            {
                distance_frequency = (2 * CENTER_TO_WALL) - distance_frequency;
                set_FSM_state(ROTATION);
            }
        }

        else if (current_state == STRING_CENTER)
        {
            distance_reached = false;
            // difference between the goal distance and the measured distance
            distance_diff = VL53L0X_get_dist_mm() - CENTER_TO_WALL;

            if (abs(distance_diff) <= TOF_PRECISION)
            {
                chprintf((BaseSequentialStream *)&SD3, "String center /n");
                distance_reached = true;
                select_color_detection(BLUE_COLOR);
                set_FSM_state(ROTATION);
            }
            chprintf((BaseSequentialStream *)&SD3, "String center %d/n", distance_diff);
            line_tracking_while_condition(distance_reached);
        }
        // 100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void)
{
    chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
