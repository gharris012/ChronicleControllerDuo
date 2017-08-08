#ifndef button_h
#define button_h

#define BUTTON_DEBOUNCE_DELAY 10
#define BUTTON_LONGPRESS_THRESHOLD 300
#include "../Adafruit_MCP23017/Adafruit_MCP23017.h"

typedef struct Button
{
    char name[10];
    uint8_t pin;
    Adafruit_MCP23017 *mcp;

    unsigned long pressTime;
    uint8_t lastState;
    uint8_t debounceState;
    unsigned long debounceTime;
} Button;

void button_onPress(Button* button);
void button_onRelease(Button* button);
void button_onClick(Button* button);
void button_onLongClick(Button* button);
void check_buttons(Button *fbuttons, uint8_t buttonCount);
void setup_buttons(Button *fbuttons, uint8_t buttonCount);

#endif
