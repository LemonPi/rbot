#include <Arduino.h>
#include <robot.h>
#include "parameters.h"
namespace robot {

// correct position using center sensor; assumes readings are up to date
void line_detect() {
	bool online = on_line(CENTER);

	if (online) ++prev_on_line;
	else {
		// false positive, not on line for enough cycles
		if (prev_on_line < CYCLES_ON_LINE) prev_on_line = 0;
		else if (counted_lines >= LINES_PER_CORRECT) {
			// -150 % 200 = -150
			int offset_x = abs((int)x) % GRID_WIDTH;
			int offset_y = abs((int)y) % GRID_WIDTH;

			// x is closer to 0, y is closer to GRID_WIDTH
			if (offset_x < offset_y) {
				// see if x is closer to 0 than y is close to GRID_WIDTH
				if (offset_x < (GRID_WIDTH - offset_y)) 
					if (x > 0) x -= offset_x;	// reset to being close to a modulo of 200
					else x += offset_x;
				else
					if (y > 0) y += GRID_WIDTH - offset_y;
					else y -= GRID_WIDTH - offset_y;
			}
			// x is closer to GRID_WIDTH, y is closer to 0
			else {
				if (offset_y < (GRID_WIDTH - offset_x))
					if (y > 0) y -= offset_y;
					else y += offset_y;
				else
					if (x > 0) x += GRID_WIDTH - offset_x;
					else x -= GRID_WIDTH - offset_x;
			}
			counted_lines = 0;
		}
		else ++counted_lines;
	}
}

bool on_line(byte pin) {
	return readings[pin] < thresholds[pin];
}

// assuing sensor input is analog and output is digital
int add_sensor(byte sensor_pin, byte indicator_pin) {
	if (sensor_num < SENSOR_MAX) {
		sensors[sensor_num] = sensor_pin;
		indicators[sensor_num] = indicator_pin;
		pinMode(indicator_pin, OUTPUT);
		++sensor_num;
	}
	return sensor_num;
}


// turn on an indicator (assuming digital) if sensor detects below threshold
void indicate_sensors() {
    for (byte i = 0; i < SENSOR_MAX; ++i) {
        readings[i] = analogRead(sensors[i]);
        if (on_line(i)) digitalWrite(indicators[i],HIGH);
        else digitalWrite(indicators[i],LOW);
 	}    
}

void calibrate() {
	if (sensor_num != SENSOR_MAX) {
		Serial.print("Not all sensors added: "); 
		Serial.println(sensor_num); 
		return;
	}

    int lows[SENSOR_MAX];
    int highs[SENSOR_MAX] = {0};
    for (byte pin = 0; pin < SENSOR_MAX; ++pin) lows[pin] = 1023;

    unsigned long calibrate_start = millis();
    // calibrate all the pins
    while ((millis() - calibrate_start) < CALLIBRATION_TIME) {
        for (byte pin = 0; pin < SENSOR_MAX; ++pin) {
        	readings[pin] = analogRead(sensors[pin]);
        	if (readings[pin] < lows[pin]) lows[pin] = readings[pin];
			else if (readings[pin] > highs[pin]) highs[pin] = readings[pin]; 
        }
    }

    // set threshold to be average (anything below is dark, anything above is bright)
    for (byte pin = 0; pin < SENSOR_MAX; ++pin) 
        thresholds[pin] = (lows[pin] + highs[pin]) / 2;

}

}