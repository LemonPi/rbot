#include <Arduino.h>
#include <robot.h>
#include "parameters.h"
namespace robot {


// correct position using center sensor; assumes readings are up to date
void line_detect() {
	// don't line correct when turning in place
	if (layers[LAYER_TURN].active) return;

	bool online = on_line(CENTER);

	if (online) {
		++cycles_on_line;
		// activate line following if close enough to target and is on a line
		if (cycles_on_line > CYCLES_CROSSING_LINE && 
			target_distance < TARGET_CLOSE && target_distance > TARGET_IMMEDIATE &&
			is_intersection((int)targets[target].x, (int)targets[target].y)) {
			// square turn in place until facing target
			int turn_to = (theta + heading_error) * RADS;
			int offset = turn_to % 90;

			// close enough to be making a hard turn
			if (abs(offset - 90) < THETA_TOLERANCE * RADS) {
				turn_to -= offset;
				// turn_to is now square
				add_target(x, y, turn_to);
				layers[LAYER_TURN].active = true;
				// skip line correction
				Serial.println("CL");
				return;
			}
		}

	}
	else {
		// false positive, not on line for enough cycles
		if (cycles_on_line < CYCLES_CROSSING_LINE && cycles_on_line >= 0) ;
		else if (counted_lines >= LINES_PER_CORRECT || cycles_on_line > CYCLES_FOLLOWING_LINE) {
			counted_lines = 0;
			// correct whichever one is closer to 0 or 200 
			correct_to_grid();

			Serial.println('C');
		}
		else {
			++counted_lines;
			Serial.println('L');
		}
		cycles_on_line = 0;
	}
}

bool on_line(byte pin) {
	return readings[pin] > thresholds[pin];
}

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

// while following a line, correct angle and heading
void correct_to_line() {
	int approx_heading = square_heading();

	int offset_x = abs((int)x) % GRID_WIDTH;
	int offset_y = abs((int)y) % GRID_WIDTH;
	// correct the other x y based on approx heading
	if ((approx_heading == DIR_UP || approx_heading == DIR_BACK || approx_heading == -180) &&
		(offset_y > LINE_WIDTH))
		y = round(y / GRID_WIDTH) * GRID_WIDTH;
	else if ((approx_heading == DIR_LEFT || approx_heading == DIR_RIGHT) &&
		(offset_x > LINE_WIDTH))
		x = round(x / GRID_WIDTH) * GRID_WIDTH;
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