
#include "Particle.h"
#include "tilt.h"

void Tilt::setTemperature(short t)
{
    int calibrated = INVALID_TEMPERATURE;
    if ( t != INVALID_TEMPERATURE )
    {
        if ( temperature_calibration_strategy == CALIBRATION_STRATEGY_TABLE )
        {
            calibrated = tableLookup(temperature_calibration_table, t, temperature_calibration_start, temperature_calibration_end, temperature_calibration_step);
        }
        else if ( temperature_calibration_strategy == CALIBRATION_STRATEGY_OFFSET )
        {
            calibrated = t + temperature_calibration_offset;
        }
        else if ( temperature_calibration_strategy == CALIBRATION_STRATEGY_NONE )
        {
            calibrated = t;
        }
    
        if ( calibrated != INVALID_TEMPERATURE )
        {
            LogTilt.info("Temperature: (%d) %d -> %d", temperature_calibration_strategy, t, calibrated);
    
            // save last temperature
            if ( tempF != last_tempF )
            {
                last_tempF = tempF;
            }
            // convert calibrated (int) to tempF (float)
            tempF = calibrated / 10.0;
            
            last_valid_read = millis();
            present = TRUE;
        }
    }
}
void Tilt::setGravity(short g)
{
    int calibrated = INVALID_GRAVITY;
    if ( gravity_calibration_strategy == CALIBRATION_STRATEGY_TABLE )
    {
        calibrated = tableLookup(gravity_calibration_table, g, gravity_calibration_start, gravity_calibration_end, gravity_calibration_step);
    }
    else if ( gravity_calibration_strategy == CALIBRATION_STRATEGY_OFFSET )
    {
        calibrated = g + gravity_calibration_offset;
    }
    else if ( gravity_calibration_strategy == CALIBRATION_STRATEGY_NONE )
    {
        calibrated = g;
    }

    if ( calibrated != INVALID_GRAVITY )
    {
        LogTilt.info("Gravity: (%d) %d -> %d", gravity_calibration_strategy, g, calibrated);
    
        // save last gravity
        if ( gravity != last_gravity )
        {
            last_gravity = gravity;
        }
        gravity = calibrated;
        
        last_valid_read = millis();
        present = TRUE;
    }
}

short Tilt::tableLookup(short *table, short input, short start, short end, short step)
{
    //int count = sizeof(table)/sizeof(table[0]);
    short count = ( ( end - start ) / step ) + 1;
    short actual = 0;
    short res = INVALID_READING;
    short i = 0;

    // LogTilt.trace("tableLookup: count: %d, input: %d, start: %d, end: %d, step: %d", count, input, start, end, step);

    if ( input > end || input < start )
    {
        res = INVALID_READING;
    }
    else
    {
        /// get closest reading without going over
        for ( i = 0 ; i < count ; i ++ )
        {
            actual = start + ( step * i );
            if ( actual == input )
            {
                res = table[i];
                break;
            }
            else if ( actual > input )
            {
                if ( i > 0 )
                {
                    short previousActual = start + ( step * ( i - 1 ) );
                    res = map(input, previousActual, actual, table[i - 1], table[i]);
                }
                else
                {
                    res = INVALID_READING;
                }
                // LogTilt.trace(" map(%d, %d, %d, %d, %d)", input, previousActual, actual, table[i - 1], table[i]);
                break;
            }
        }
    }

    LogTilt.trace("Lookup result: %d -> %d", input, res);

    return res;
}
