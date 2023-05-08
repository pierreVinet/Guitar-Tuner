#include <ch.h>
#include <hal.h>
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <motors.h>

#include "line_tracking.h"
#include "process_image.h"
// include main to include stdlib.h for the abs() function
#include "main.h"

static bool enabled_motors = false;

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

    int16_t speed = 0;
    int16_t speed_correction = 0;

    while (1)
    {

        time = chVTGetSystemTime();
        if (get_FSM_state() == LINE_TRACKING)
        {

            // search for a line (0) not found, (1) found
            if (!line_detected)
            {
                line_detected = get_line_detection();
            }
            // computes the speed to give to the motors
            // distance_cm is modified by the image processing thread
            // speed = pi_regulator(get_distance_cm(), GOAL_DISTANCE);

            // motors enables and line found -> follow the line
            if (line_detected && enabled_motors)
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
                right_motor_set_speed(100 - ROTATION_COEFF * speed_correction);
                left_motor_set_speed(100 + ROTATION_COEFF * speed_correction);
            }
            // motors enabled but no line found -> e-puck rotates until finds a line
            else if (!line_detected && enabled_motors)
            {
// choose to rotate clockwise
#if ROTATION_CLOCKWISE
                right_motor_set_speed(-100);
                left_motor_set_speed(100);
#endif

// choose to rotate anticlockwise
#if !ROTATION_CLOCKWISE
                right_motor_set_speed(100);
                left_motor_set_speed(-100);

#endif
            }
            // motors disabled
            else
            {
                // stop the motors
                right_motor_set_speed(0);
                left_motor_set_speed(0);
            }
        }
        // 100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void)
{
    chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}

void set_enabled_motors(bool enable)
{
    enabled_motors = enable;
}

void toogle_enabled_motors()
{
    enabled_motors = !enabled_motors;
}