#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

// Specify the 2 consecutive lines used for tracking the black line
// The line number starts from 0 and ending to PO8030_MAX_HEIGHT - 1. Consult camera/po8030.h
// But as 2 lines will be used, the value of the first line can be higher than PO8030_MAX_HEIGHT - 2
#define USED_LINE 200 // Must be inside [0..478], according to the above explanations

// number of pixel of a line of the captured image
#define IMAGE_BUFFER_SIZE 640
// width of the slope of pixel intensity (in pixel)
#define WIDTH_SLOPE 5
// min width of the line detected (in pixel)
#define MIN_LINE_WIDTH 80

// RGB LED used for interaction with plotImage Python code
#define USED_RGB_LED LED4
#define INTENSITY_RGB_LED 10
#define PXTOCM 1570.0f // experimental value
#define GOAL_DISTANCE 10.0f
#define MAX_DISTANCE 25.0f

// List of detection color
typedef enum
{
    RED_COLOR,
    GREEN_COLOR,
    BLUE_COLOR,
} color_detection_t;

uint8_t get_line_detection(void);
uint16_t get_line_position(void);
void process_image_start(void);
void select_color_detection(color_detection_t choice_detect_color);

#endif /* PROCESS_IMAGE_H */