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

typedef enum
{
    PERPENDICULAR_CALIBRATION = 0,
    FREQUENCY_DETECTION

} FSM_STATE;

FSM_STATE get_FSM_state(void);
void set_FSM_state(FSM_STATE new_state);

#endif
