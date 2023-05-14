#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <camera/dcmi_camera.h>
#include <msgbus/messagebus.h>
#include <parameter/parameter.h>

    /** Robot wide IPC bus. */
    extern messagebus_t bus;

    extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

// Different states of the FSM
typedef enum
{
    FREQUENCY_DETECTION = 0,
    STRING_POSITION,
    ROTATION,
    FREQUENCY_POSITION,
    STRING_CENTER,
    DO_NOTHING
} FSM_STATE;

// frequency for each string of the guitar and their respective range
#define SIXTH_STRING_FREQ_MIN 76
#define SIXTH_STRING_FREQ 82.41f
#define SIXTH_STRING_FREQ_MAX 88

#define FIFTH_STRING_FREQ_MIN 103
#define FIFTH_STRING_FREQ 110.00f
#define FIFTH_STRING_FREQ_MAX 117

#define FOURTH_STRING_FREQ_MIN 138
#define FOURTH_STRING_FREQ 146.83f
#define FOURTH_STRING_FREQ_MAX 156

#define THIRD_STRING_FREQ_MIN 184
#define THIRD_STRING_FREQ 196.00f
#define THIRD_STRING_FREQ_MAX 208

#define SECOND_STRING_FREQ_MIN 232
#define SECOND_STRING_FREQ 246.94f
#define SECOND_STRING_FREQ_MAX 262

#define FIRST_STRING_FREQ_MIN 310
#define FIRST_STRING_FREQ 329.63f
#define FIRST_STRING_FREQ_MAX 350

FSM_STATE get_FSM_state(void);
FSM_STATE get_FSM_previous_state(void);
void set_FSM_state(FSM_STATE new_state);
void increment_FSM_state(void);

#endif /* MAIN_H */
