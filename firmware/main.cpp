// == [ includes ] ==
#include <stdarg.h>
#include "Particle.h"
#include "config.h"
#include "keys.h"

SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(AUTOMATIC);
BLE_SETUP(DISABLED);

SerialLogHandler logHandler(LOG_LEVEL_WARN, {
    { "app", LOG_LEVEL_INFO },
    { "app.ble", LOG_LEVEL_INFO },
    { "app.sensor.tilt", LOG_LEVEL_INFO }
});

Logger LogBle("app.ble");
Logger LogTilt("app.sensor.tilt");

// == [ setup ] ==

// ===  PURPLE TILT ===
Tilt tilt_purple = { "purple", 0x40, 23, 25, 27, CALIBRATION_STRATEGY_NONE, CALIBRATION_STRATEGY_NONE, &LogTilt }; // name, color, blynk, temp, gravity

// ===  ORANGE TILT ===
Tilt tilt_orange = { "orange", 0x50, 22, 24, 26, CALIBRATION_STRATEGY_NONE, CALIBRATION_STRATEGY_NONE, &LogTilt }; // name, color, blynk, temp, gravity

unsigned long int check_memory_next_time = 0;
const int check_memory_delay = 5000;
unsigned long int check_tilt_next_time = 0;
const int check_tilt_delay = 5000;

// five seconds
const int update_blynk_delay = 5000;
unsigned long int update_blynk_next_time = update_blynk_delay;

void setup() {
    Serial.begin(115200);
    delay(2000); // allow time to connect serial monitor

    Log.info("System started");
    Log.info("Device ID: %s", (const char*)System.deviceID());
    Log.info("System version: %s", (const char*)System.version());
    Log.info("App version: %s", (const char*)APP_VERSION);
    ppublish("Starting up App Version: %s", (const char*)APP_VERSION);

    Log.info("setting up blynk");
    Blynk.begin(BLYNK_KEY);

    Log.info("setting up ble scanner");
    ble_scanner_setup(&tilt_callback, &LogBle);

    Log.info("Ready.");
}

bool rescanOWN = FALSE;
void loop()
{
    Blynk.run();

    if ( millis() > check_memory_next_time )
    {
        check_memory();
        check_memory_next_time += check_memory_delay;
    }
    if ( millis() > check_tilt_next_time )
    {
        tilt_purple.checkConnection();
        tilt_orange.checkConnection();
        check_tilt_next_time += check_tilt_delay;
    }
    if ( millis() > update_blynk_next_time )
    {
        update_blynk();
        update_blynk_next_time += update_blynk_delay;
    }
}

void check_memory()
{
    Log.info("Free memory: %ld", System.freeMemory());
}

void update_blynk()
{
    char buf[6];
    // send gravity
    if ( tilt_purple.gravity != INVALID_GRAVITY )
    {
        if ( tilt_purple.blynkGravityPin >= 0 )
        {
            snprintf(buf, 6, "%0.03f", tilt_purple.gravity / 1000.0);
            Blynk.virtualWrite(tilt_purple.blynkGravityPin, buf);
        }
        if ( tilt_purple.blynkGravityGraphPin >= 0 )
        {
            Blynk.virtualWrite(tilt_purple.blynkGravityGraphPin, tilt_purple.gravity);
        }
    }
    if ( tilt_purple.tempF != INVALID_TEMPERATURE )
    {
        if ( tilt_purple.blynkTempPin >= 0 )
        {
            snprintf(buf, 6, "%0.01f", tilt_purple.tempF);
            Blynk.virtualWrite(tilt_purple.blynkTempPin, buf);
        }
    }

    if ( tilt_orange.gravity != INVALID_GRAVITY )
    {
        if ( tilt_orange.blynkGravityPin >= 0 )
        {
            snprintf(buf, 6, "%0.03f", tilt_orange.gravity / 1000.0);
            Blynk.virtualWrite(tilt_orange.blynkGravityPin, buf);
        }
        if ( tilt_orange.blynkGravityGraphPin >= 0 )
        {
            Blynk.virtualWrite(tilt_orange.blynkGravityGraphPin, tilt_orange.gravity);
        }
    }
    if ( tilt_orange.tempF != INVALID_TEMPERATURE )
    {
        if ( tilt_orange.blynkTempPin >= 0 )
        {
            snprintf(buf, 6, "%0.01f", tilt_orange.tempF);
            Blynk.virtualWrite(tilt_orange.blynkTempPin, buf);
        }
    }
}

void tilt_callback(short color_id, short temperature, short gravity)
{
    LogTilt.info("Tilt: %#04X => temp: %d ; gravity: %d", color_id, temperature, gravity);
    if ( color_id == tilt_purple.color_id )
    {
        tilt_purple.setTemperature(temperature);
        tilt_purple.setGravity(gravity);
    }
    else if ( color_id == tilt_orange.color_id )
    {
        tilt_orange.setTemperature(temperature);
        tilt_orange.setGravity(gravity);
    }
}

void ppublish(String message, ...)
{
    char buffer[50];
    va_list args;
    va_start(args, message);
    vsnprintf(buffer, 50, message.c_str(), args);
    Particle.publish("duo", buffer);

    va_end(args);
}
