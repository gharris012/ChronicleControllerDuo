#ifndef config_h
#define config_h

#define APP_VERSION "d2.0.1"

// One-wire network pin
#define OWNPIN D6
#define LCDLINELENGTH 10
#define LCDLINEHEIGHT 16
#define LCDBLANKLINE "          "
#define AUTO_MODE_ON 1
#define AUTO_MODE_OFF 2
#define AUTO_MODE_PID 3
#define AUTO_MODE_AUTO 4
#define MENU_ON AUTO_MODE_ON
#define MENU_OFF AUTO_MODE_OFF
#define MENU_PID AUTO_MODE_PID       // PID control
#define MENU_AUTO AUTO_MODE_AUTO     // Dumb auto : setpoint, threshold min/max
#define CONTROL_HIGH_DIFFERENTIAL 10 // error > DIFFERENTIAL -> high differential
#define BUTTON_COUNT 6
#define INVALID_TEMPERATURE -123.0
#define INVALID_GRAVITY -123
#define INVALID_READING -123

#define CALIBRATION_STRATEGY_NONE 0
#define CALIBRATION_STRATEGY_OFFSET 1
#define CALIBRATION_STRATEGY_TABLE 2

#include "Particle.h"
#include <math.h>
#include <Blynk/blynk.h>
#include "ble_duo/ble_duo.h"
#include "tilt/tilt.h"

void check_memory();
void update_blynk();
void tilt_callback(short color_id, short temperature, short gravity);

void ppublish(String message, ...);
#endif
