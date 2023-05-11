#ifndef LINE_TRACKING_H
#define LINE_TRACKING_H

#define ROTATION_THRESHOLD 10
#define ROTATION_COEFF 1
#define ERROR_THRESHOLD 0.1f //[cm] because of the noise of the camera
#define KP 800.0f
#define KI 3.5f // must not be zero
#define MAX_SUM_ERROR (MOTOR_SPEED_LIMIT / KI)
#define ROTATION_CLOCKWISE 0 // 1 = rotation clockwise, 0 = rotation anticlockwise

#define NSTEP_ONE_TURN 1000  // number of step for 1 turn of the motor
#define NSTEP_ONE_EL_TURN 4  // number of steps to do 1 electrical turn
#define NB_OF_PHASES 4       // number of phases of the motors
#define WHEEL_PERIMETER 13   // [cm]
#define WHEELS_DISTANCE_CM 7 // [cm]

#define SPEED_STEPS_PER_SECOND 600
#define TIME_CONVERSION_FACTOR 1000000

// start the PI regulator thread
void pi_regulator_start(void);

#endif /* PI_REGULATOR_LINE_TRACKING_H */