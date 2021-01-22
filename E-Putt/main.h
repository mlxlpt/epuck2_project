#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

//These defines are used by multiples source files.
#define TIME_MS_PIDREG 			10 //execution period of the controller.
#define IMAGE_BUFFER_SIZE		640 //total width of camera, pixels

//States available. Keep LED states at the end of the list. LEDSTATE is used to identify them (IDs are above)
enum eputtState{STARTUP = 0, MANUAL_MOVE, SEARCH_BALL, BALL_LOCKED, CHARGE_BALL,
				LEDSTATE, LED_MV_SB, LED_BALL_NF, LED_SUCCESS};

void setState(enum eputtState nouvEtat);
enum eputtState getState(void);
void switchState(bool success);
bool led_handler(void);
void stateLed_update(void);

/** Robot wide IPC bus. */
extern messagebus_t bus;
extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

#endif
