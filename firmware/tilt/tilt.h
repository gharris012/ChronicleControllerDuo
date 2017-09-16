#ifndef TILT_H
#define TILT_H

#include "Particle.h"

// if we haven't gotten a valid reading in this amount of time, mark readings invalid
#define TILT_GRACE_PERIOD 600000
#define CALIBRATION_STRATEGY_NONE 0
#define CALIBRATION_STRATEGY_OFFSET 1
#define CALIBRATION_STRATEGY_TABLE 2

#ifndef INVALID_READING
    #define INVALID_TEMPERATURE -123.0
    #define INVALID_GRAVITY -123
    #define INVALID_READING -123
#endif

typedef struct Tilt
{
    const char *name;
    const short color_id;
    byte blynkPin = -1;

    float tempF = INVALID_READING;
    short gravity = INVALID_READING; // 1010 -> 1.010

    byte temperature_calibration_strategy = CALIBRATION_STRATEGY_NONE;
    short temperature_calibration_offset = 0;
    short temperature_calibration_start = 0;
    short temperature_calibration_end = 0;
    byte temperature_calibration_step = 0;
    unsigned short *temperature_calibration_table;

    byte gravity_calibration_strategy = CALIBRATION_STRATEGY_NONE;
    short gravity_calibration_offset = 0;
    short gravity_calibration_start = 0;
    short gravity_calibration_end = 0;
    short gravity_calibration_step = 0;
    unsigned short *gravity_calibration_table;

    float last_tempF = INVALID_TEMPERATURE;
    float last_gravity = INVALID_GRAVITY;
    unsigned long int last_valid_read = 0;
    bool present = FALSE;

    Logger *logger;

    Tilt(const char *n, short c, byte ts, byte gs) : name(n), color_id(c), temperature_calibration_strategy(ts), gravity_calibration_strategy(gs) {};
    Tilt(const char *n, short c, byte b, byte ts, byte gs) : name(n), color_id(c), blynkPin(b), temperature_calibration_strategy(ts), gravity_calibration_strategy(gs) {};
    Tilt(const char *n, short c, byte b, byte ts, byte gs, Logger *logger) : name(n), color_id(c), blynkPin(b), temperature_calibration_strategy(ts), gravity_calibration_strategy(gs), logger(logger) {};

    void setTemperature(short);
    void setGravity(short);

    void checkConnection();

    short tableLookup(unsigned short*, short, short, short, short);
} Tilt;

#endif
