#ifndef LINE_TRACKING_H
#define LINE_TRACKING_H

#define ROTATION_THRESHOLD 10
#define ROTATION_COEFF 1
#define ERROR_THRESHOLD 0.1f //[cm] because of the noise of the camera
#define KP 800.0f
#define KI 3.5f // must not be zero
#define MAX_SUM_ERROR (MOTOR_SPEED_LIMIT / KI)
#define ROTATION_CLOCKWISE 0 // 1 = rotation clockwise, 0 = rotation anticlockwise

// start the PI regulator thread
void pi_regulator_start(void);

// handle motors
void set_enabled_motors(bool);
void toogle_enabled_motors(void);

#endif /* PI_REGULATOR_LINE_TRACKING_H */