#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include <usbcfg.h>
#include <camera/po8030.h>

#include "main.h"
#include "image_processing.h"

// Specify the 2 consecutive lines used for tracking the line
#define USED_LINE 200 // Must be inside [0..478], according to the above explanations
// width of the slope of pixel intensity (in pixel)
#define WIDTH_SLOPE 5
// minimum width of the line detected (in pixel)
#define MIN_LINE_WIDTH 80

// position of the line detected (center, left, right ...)
static uint16_t line_position = IMAGE_BUFFER_SIZE / 2; // center by default
static bool line_found = false;

// Extracts only the pixels of the color specified.
static color_detection_t detect_color = RED_COLOR;

// semaphore to indicate when an image is ready
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

/*
 *  Returns true if a line is found, otherwise false.
 */
bool get_line_detection(void)
{
    return line_found;
}

/*
 *  Returns the position in pixel of the line detected.
 */
uint16_t get_line_position(void)
{
    return line_position;
}

/*
 *  Set the color to detect.
 *
 *  params:
 *  color_detection_t choice_detect_color      Color to detect.
 */
void select_color_detection(color_detection_t choice_detect_color)
{
    detect_color = choice_detect_color;
}

/*
 *  Update the static variable "line_position" to true (1) if a line is detected
 *  or false (0) if no line wider than MIN_LINE_WIDTH is detected.
 *  A line is detected performing an average of the intensity of all the pixels
 *  of the buffer. Then each pixel is compared to the mean until two pixel are
 *  at the opposite side of the mean at a distance of WIDTH_SLOPE from each other.
 *  Therefore a descending and ascending slope can be detected, implying that
 *  the camera detected a line of lower intensity compared to the mean.
 *
 *  params:
 *  uint8_t *buffer         Pointer to the buffer containing the intensity of
 *                          the pixel of the detected color.
 */
void line_detection(uint8_t *buffer)
{
    uint16_t i = 0, begin = 0, end = 0;
    uint8_t stop = 0, wrong_line = 0;
    // mean intensity of all the pixels (IMAGE_BUFFER_SIZE)
    uint32_t mean = 0;

    // performs an average (intensity pixel average)
    for (uint16_t i = 0; i < IMAGE_BUFFER_SIZE; i++)
    {
        mean += buffer[i];
    }
    mean /= IMAGE_BUFFER_SIZE;

    do
    {
        line_found = 0;
        wrong_line = 0;
        // search for a begin (search for an descending slope)
        while (stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
        {
            // the slope must at least be WIDTH_SLOPE wide
            // the beginning of the slope must be higher in intensity than the mean, and the end of the slope lower
            if (buffer[i] > mean && buffer[i + WIDTH_SLOPE] < mean)
            {
                begin = i;
                stop = 1;
            }
            i++;
        }
        // if a begin (descending slope) was found, search for an end (ascending slope)
        if (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin)
        {
            stop = 0;

            while (stop == 0 && i < IMAGE_BUFFER_SIZE)
            {
                // the slope must at least be WIDTH_SLOPE wide
                // the beginning of the slope must be lower in intensity than the mean, and the end of the slope higher
                if (i > WIDTH_SLOPE && buffer[i] > mean && buffer[i - WIDTH_SLOPE] < mean)
                {
                    end = i;
                    stop = 1;
                    line_found = 1;
                }
                i++;
            }
        }

        // if a line too small has been detected, continues the search
        if (line_found && (end - begin) < MIN_LINE_WIDTH)
        {
            i = end;
            begin = 0;
            end = 0;
            stop = 0;
            wrong_line = 1;
        }
    } while (wrong_line);

    if (line_found)
    {
        // if a line is found, line_position is update to the position of the line
        line_position = (begin + end) / 2;
    }
    else
    {
        // if no line is found, line_position is update to it's initial value
        line_position = IMAGE_BUFFER_SIZE / 2; // center
    }
}

/*
 *  Thread that calls functions from dcmi_camera.c to capture the intensity of
 *  pixels from two lines of the camera. The pixels intensity is stored in RGB565
 *  format. The thread uses a semaphore to communicate with the ProcessImage
 *  thread indicating that an image is ready.
 */
static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg)
{

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    // Captures pixels from 0 to IMAGE_BUFFER_SIZE of the lines USED_LINE and USED_LINE + 1 in RGB565 format
    po8030_advanced_config(FORMAT_RGB565, 0, USED_LINE, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
    dcmi_enable_double_buffering();
    dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
    dcmi_prepare();

    while (1)
    {
        FSM_STATE current_state = get_FSM_state();
        if (current_state == STRING_POSITION || current_state == FREQUENCY_POSITION)
        {
            dcmi_capture_start();
            wait_image_ready();
            // signals an image has been captured
            chBSemSignal(&image_ready_sem);
        }
        chThdSleepMilliseconds(200);
    }
}

/*
 *  Thread that process only one line of the image captured. Depending
 *  of wich color we want to detect, this thread process only the intensity
 *  of the pixels of the color chosen.
 */
static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg)
{
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    // pointer to the address filled with the last image in RGB565
    uint8_t *img_buff_ptr;
    // array containing only the intensity of the pixel of the color chosen
    uint8_t image[IMAGE_BUFFER_SIZE] = {0}; // initialized to 0

    while (1)
    {
        chBSemWait(&image_ready_sem);
        // gets the pointer to the array filled with the last image in RGB565
        img_buff_ptr = dcmi_get_last_image_ptr();

        switch (detect_color)
        {
        case RED_COLOR:
            // Extracts only the red pixels
            for (uint16_t i = 0; i < (2 * IMAGE_BUFFER_SIZE); i += 2)
            {
                image[i / 2] = (uint8_t)img_buff_ptr[i] & 0xF8;
            }
            break;
        case GREEN_COLOR:
            // Extracts only the green pixels
            for (uint16_t i = 0; i < (2 * IMAGE_BUFFER_SIZE); i += 2)
            {
                // extracts 3 LSbits of the first byte and the 3 MSbits of second byte
                image[i / 2] = (((uint8_t)img_buff_ptr[i] & 0x07) << 5) + (((uint8_t)img_buff_ptr[i + 1] & 0xE0) >> 3);
            }
            break;
        case BLUE_COLOR:
            // Extracts only the blue pixels
            for (uint16_t i = 0; i < (2 * IMAGE_BUFFER_SIZE); i += 2)
            {
                // extracts 5 LSbits of the LSByte (Second byte in big-endian format)
                image[i / 2] = ((uint8_t)img_buff_ptr[i + 1] & 0x1F) << 3;
            }
            break;
        }

        line_detection(image);
    }
}

/*
 *  Starts the threads CaptureImage and ProcessImage.
 */
void image_processing_start(void)
{
    chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
    chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
}
