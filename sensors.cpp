#include <Arduino.h>
#include <robot.h>
#include "parameters.h"
namespace robot {


// teleport to the nearest line, holding the farther away position value constant
void correct_to_grid() {
	// -150 % 200 = -150
	int offset_x = abs((int)x) % GRID_WIDTH;
	int offset_y = abs((int)y) % GRID_WIDTH;

	bool correct_x;
	if (offset_x < offset_y) {
		if (offset_x - 0 < GRID_WIDTH - offset_y) correct_x = true; 
		else correct_x = false;
	}
	else {
		if (offset_y - 0 < GRID_WIDTH - offset_x) correct_x = false;
		else correct_x = true;
	}
	// if not correct x, correct y
	if (correct_x) {
		x = round(x / GRID_WIDTH) * GRID_WIDTH;
		// leaving line forward
		if (theta > -HALFPI && theta < HALFPI) x += HALF_LINE_WIDTH;
		else x -= HALF_LINE_WIDTH;
	}
	else {
		y = round(y / GRID_WIDTH) * GRID_WIDTH;
		// leaving line right
		if (theta > 0 && theta < PI) y += HALF_LINE_WIDTH;
		else y -= HALF_LINE_WIDTH;
	}
}

// round angle to multiples of 90
int square_heading() {
	int approx_heading = theta * RADS;	// degrees between [-180,180]
	int offset = approx_heading % 90;	// offset from a 90 degree turn
	// force heading to be modulo 90 to give direction
	approx_heading -= offset;
	// offset large enough to consider correcting
	theta = approx_heading * DEGS;
	return approx_heading;
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

bool on_line(byte pin) {
	return on_lines & pin;
}

// turn on an indicator (assuming digital) if sensor detects below threshold
void indicate_sensors() {
    for (byte i = 0; i < SENSOR_MAX; ++i) {
    	// clear ith bit or set ith bit
    	if (analogRead(sensors[i]) > thresholds[i]) {on_lines |= (1 << i); digitalWrite(indicators[i],HIGH);}
    	else										{on_lines &= ~(1 << i); digitalWrite(indicators[i],LOW);}
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
    int readings[SENSOR_MAX];
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
        thresholds[pin] = (lows[pin] + highs[pin]) / THRESHOLD_TOLERANCE;
}

}