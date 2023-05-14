#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

// number of pixel captured by the image for each line
#define IMAGE_BUFFER_SIZE 640

// List of detection color
typedef enum
{
    RED_COLOR,
    GREEN_COLOR,
    BLUE_COLOR,
} color_detection_t;

bool get_line_detection(void);
uint16_t get_line_position(void);
void select_color_detection(color_detection_t choice_detect_color);
void image_processing_start(void);

#endif /* PROCESS_IMAGE_H */