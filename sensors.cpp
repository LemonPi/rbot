#include <Arduino.h>
#include <robot.h>
#include "parameters.h"
namespace robot {


// correct position using center sensor; assumes readings are up to date
void line_detect() {
	// don't line correct when turning in place
	if (!on || layers[LAYER_TURN].active) return;

	bool online = on_line(CENTER);

	// just deviated from a line that the robot was following
	if (deviate_from_line) {
		if (online || on_line(LEFT) || on_line(RIGHT)) {
			cycles_on_line = 0;
			deviate_from_line = false;
			// no more correction
			if (online) return;
			// total distance accumulated while veering off course
			float deviate_distance = tot_distance - pre_deviate_distance;
			float deviate_theta = atan2(SIDE_SENSOR_DISTANCE, deviate_distance);

			// veered on right = 1; veered on left = -1
			// distances to correct for relative to line just left
			float perpendicular_correct = SIDE_SENSOR_DISTANCE * cos(deviate_theta);
			float parallel_correct = SIDE_SENSOR_DISTANCE * sin(deviate_theta);
			// veering on the right
			if (on_line(LEFT)) {
				theta += deviate_theta; 
				// correct for position
				if 		(theta < -HALFPI) 	{y -= perpendicular_correct; x += parallel_correct;}
				else if (theta < 0) 		{x += perpendicular_correct; y += parallel_correct;}
				else if (theta < HALFPI) 	{y += perpendicular_correct; x -= parallel_correct;}
				else if (theta < PI) 		{x -= perpendicular_correct; y -= parallel_correct;}
				Serial.println("DR");
			}
			else if (on_line(RIGHT)) {
				theta -= deviate_theta;
				if 		(theta < -HALFPI) 	{x -= perpendicular_correct; y += parallel_correct;}
				else if (theta < 0) 		{y -= perpendicular_correct; x -= parallel_correct;}
				else if (theta < HALFPI) 	{x += perpendicular_correct; y -= parallel_correct;}
				else if (theta < PI) 		{y -= perpendicular_correct; x += parallel_correct;}
				Serial.println("DL");
			}
		}
		return;	// skip the usual line correct
	}

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
		// following a line, correct both theta and the horizontal position
		if (cycles_on_line > CYCLES_FOLLOWING_LINE && 
			(cycles_on_line - CYCLES_CROSSING_LINE) % CYCLES_PER_CORRECT == 0) {
			// correct to grid using the other 2 sensors
			side_correct |= on_line(LEFT);			// 0001 or 0000 binary
			side_correct |= on_line(RIGHT) << 1;	// 0010 or 0000 in binary
			if (side_correct & B1 == B1) side_correct |= !on_line(LEFT) << 2;		// 0100 or 0000
			if (side_correct & B10 == B10) side_correct |= !on_line(RIGHT) << 3;	// 1000 or 0000
			// left and right have activated and one of them passed
			if (side_correct == B1011 || side_correct == B0111) {		
				correct_to_grid();
				side_correct = 0;
			}

			correct_to_line();
			Serial.println('F');
		}
	}
	else {
		// false positive, not on line for enough cycles
		if (cycles_on_line < CYCLES_CROSSING_LINE && cycles_on_line >= 0) ;
		// was following line, just veered off course
		else if (cycles_on_line > CYCLES_FOLLOWING_LINE || cycles_on_line < 0) {
			// first time off line
			if (cycles_on_line > 0) {cycles_on_line = 0; pre_deviate_distance = tot_distance;}
			cycles_on_line -= 2;

			if (cycles_on_line < -CYCLES_DEVIATE_LINE) { deviate_from_line = true; Serial.println("LL");}
			// int approx_heading = theta * RADS;	// degrees between [-180,180]
			// int offset = approx_heading % 90;	// offset from a 90 degree turn
			// // amplify the offset
			// theta = (approx_heading + OFFSET_GAIN*offset) * DEGS;
			
			// // force heading to be modulo 90 to give direction
			// approx_heading -= offset;

			// // account for veering off line by line width
			// if (0 < approx_heading <= 45        || 135 < approx_heading <= 180) y += LINE_WIDTH;
			// else if (45 < approx_heading <= 90  || -90 < approx_heading <= -45) x += LINE_WIDTH;
			// else if (90 < approx_heading <= 135 || -135 < approx_heading <= -90) x -= LINE_WIDTH;
			// else if (-45 < approx_heading <= 0  || -180 < approx_heading <= -135) y -= LINE_WIDTH;

			
			return;	// skip reset to 0
		}
		else if (counted_lines >= LINES_PER_CORRECT) {
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

void correct_to_line() {
	int approx_heading = theta * RADS;	// degrees between [-180,180]
	int offset = approx_heading % 90;	// offset from a 90 degree turn
	// force heading to be modulo 90 to give direction
	approx_heading -= offset;
	// offset large enough to consider correcting
	if (abs(offset - 90) > 2 * THETA_TOLERANCE * RADS) {
		theta = approx_heading * DEGS;
	}

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

    digitalWrite(indicators[CENTER], HIGH);

}

}