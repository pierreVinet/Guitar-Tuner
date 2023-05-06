#ifndef CALIBRATION_TOF_H
#define CALIBRATION_TOF_H

bool calibration(float dist);
void robot_rotate(float pas, bool clockwise);

// initialize the thread for the perpendicular calibration
void perpendicular_calibration_start(void);

#endif /* PROCESS_IMAGE_H */