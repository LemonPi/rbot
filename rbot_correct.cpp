#include "rbot.h"
#include "parameters.h"

namespace robot {

// correct passing a line not too close to an intersection
void passive_correct() {
	// activate passive correct when either left or right sensor FIRST ACTIVATES
	if ((on_line(LEFT) || on_line(RIGHT)) && passive_status == PASSED_NONE && far_from_intersection(x, y)) {

		correct_initial_distance = tot_distance;
		// only look at RIGHT LEFT CENTER
		passive_status |= (on_lines & B111);
		Serial.print('p');
		Serial.println(passive_status, BIN);
		return;
	}

	if (passive_status == PASSED_NONE || !far_from_intersection(x, y)) return;

	// travelling too parallel to line
	if (tot_distance - correct_initial_distance > CORRECT_TOO_FAR) {
		passive_status = PASSED_NONE;
		Serial.print("pR");
		return;
	}

	// check if center one first activated; halfway there
	if (on_line(CENTER) && !(passive_status & CENTER)) correct_half_distance = tot_distance;

	// check if encountering any additional lines
	passive_status |= (on_lines & B111);

	// passing each line
	if (!on_line(LEFT) && (passive_status & LEFT)) passive_status |= PASSED_LEFT;
	if (!on_line(RIGHT) && (passive_status & RIGHT)) passive_status |= PASSED_RIGHT;

	// correct when 1 fully passed and the other one just activated
	if (((passive_status & PASSED_LEFT_RIGHT) == PASSED_LEFT_RIGHT) ||
		((passive_status & PASSED_RIGHT_LEFT) == PASSED_RIGHT_LEFT)) {

		// distance since when passive correct was activated
		float correct_elapsed_distance;
		// under 1 behaviour loop
		if (tot_distance == correct_initial_distance) {
			instant_tick_l = tick_l;
			instant_tick_r = tick_r;
			double displacement_l = dir_l * (double)instant_tick_l * MM_PER_TICK_L;
			double displacement_r = dir_r * (double)instant_tick_r * MM_PER_TICK_R;
			correct_elapsed_distance = (displacement_l + displacement_r) * 0.5;
		}
		else correct_elapsed_distance = tot_distance - correct_initial_distance;
		
		// correct only if the 2 half distances are about the same
		if (abs((tot_distance - correct_half_distance) - (correct_half_distance - correct_initial_distance)) < CORRECT_CROSSING_TOLERANCE) {
			
			// always positive
			float theta_offset = atan2(correct_elapsed_distance, SIDE_SENSOR_DISTANCE);

			// reverse theta correction if direction is backwards
			if (layers[get_active_layer()].speed < 0) theta_offset = -theta_offset; 

			Serial.println(square_heading());
			// assume whichever one passed first was the first to hit
			if (passive_status & PASSED_LEFT) theta = (square_heading()*DEGS) + theta_offset;
			else theta = (square_heading()*DEGS) - theta_offset;

			Serial.println(theta_offset);
			Serial.print('P');
			Serial.println(passive_status, BIN);

		}
		// suspicious of an intersection
		else {
			Serial.println("pI");
		}

		// reset even if not activated on this line (false alarm)
		passive_status = PASSED_NONE;
	}

}

bool far_from_intersection(int xx, int yy) {
	byte offset_x = abs((int)xx) % GRID_WIDTH;
	byte offset_y = abs((int)yy) % GRID_WIDTH;
	return (offset_x < INTERSECTION_TOO_CLOSE || offset_x > GRID_WIDTH - INTERSECTION_TOO_CLOSE) ^
			(offset_y < INTERSECTION_TOO_CLOSE || offset_y > GRID_WIDTH - INTERSECTION_TOO_CLOSE);	
}

void passive_position_correct() {


	if (on_line(CENTER)) {
		++cycles_on_line;
		// activate line following if close enough to target and is on a line
	}
	else {
		// false positive, not on line for enough cycles
		if (cycles_on_line < CYCLES_CROSSING_LINE && cycles_on_line >= 0) ;
		else if (counted_lines >= LINES_PER_CORRECT && far_from_intersection(x, y)) {
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

}	// end namespace