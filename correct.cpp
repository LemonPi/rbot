#include "rbot.h"
#include "parameters.h"

namespace robot {

// correct passing a line not too close to an intersection
void passive_correct() {

	// still on cool down
	if (passive_status < PASSED_NONE) {++passive_status; return;}

	if (!far_from_intersection(x, y)) {
		if (passive_status > PASSED_NONE) passive_status = PASSED_NONE;
		return;
	}

	// SERIAL_PRINTLN(passive_status, BIN);
	// check if center one first activated; halfway there
	if (on_line(CENTER) && !(passive_status & CENTER)) {
		correct_half_distance = current_distance();
		SERIAL_PRINTLN("PH");
	}


	// activate passive correct when either left or right sensor FIRST ACTIVATES
	if ((on_line(LEFT) || on_line(RIGHT)) && passive_status == PASSED_NONE) {
		correct_initial_distance = current_distance();
		// only look at RIGHT LEFT CENTER
		passive_status |= (on_lines & ENCOUNTERED_ALL);
		SERIAL_PRINT("PS");
		SERIAL_PRINTLN(passive_status, BIN);

		// all hit at the same time, don't know heading
		if (passive_status == ENCOUNTERED_ALL) hit_first = CENTER;
		else if (passive_status & LEFT) hit_first = LEFT;
		else hit_first = RIGHT;
		
		// return;
	}

	if ((passive_status & LEFT) && !on_line(LEFT)) passive_status |= PASSED_LEFT;
	if ((passive_status & RIGHT) && !on_line(RIGHT)) passive_status |= PASSED_RIGHT;

	// check if encountering any additional lines
	passive_status |= (on_lines & ENCOUNTERED_ALL);

	// distance from first to center too far, probably too parallel to line
	if (passive_status > PASSED_NONE && !(passive_status & CENTER) && (current_distance() - correct_initial_distance > CORRECT_TOO_FAR)) {
		passive_status = PASSED_COOL_DOWN;
		SERIAL_PRINT("PP");
		SERIAL_PRINTLN(current_distance() - correct_initial_distance);
		return;
	}
	// already hit center, see if second half distance is too far from first half distance
	else if ((passive_status & CENTER) && 
		((current_distance() - correct_half_distance) >	// second half distance
		(correct_half_distance - correct_initial_distance + CORRECT_CROSSING_TOLERANCE))) { // first half distance plus some room for error

		SERIAL_PRINT("PD");
		SERIAL_PRINT(correct_half_distance - correct_initial_distance);
		SERIAL_PRINT(' ');
		SERIAL_PRINTLN(current_distance() - correct_half_distance);
		passive_status = PASSED_COOL_DOWN;
	} 
				
	// correct at the first encounter of line for each sensor
	if ((passive_status & ENCOUNTERED_ALL) == ENCOUNTERED_ALL) {

		// correct only if the 2 half distances are about the same
		if (abs((current_distance() - correct_half_distance) - (correct_half_distance - correct_initial_distance)) < CORRECT_CROSSING_TOLERANCE) {
			
			// distance since when passive correct was activated
			float correct_elapsed_distance = current_distance() - correct_initial_distance;
			// always positive
			float theta_offset = atan2(correct_elapsed_distance, SIDE_SENSOR_DISTANCE);

			// reverse theta correction if direction is backwards
			if (layers[get_active_layer()].speed < 0) theta_offset = -theta_offset; 
			float theta_candidate;
			// assume whichever one passed first was the first to hit
			if (passive_status & PASSED_LEFT) theta_candidate = (square_heading()*DEGS) + theta_offset;
			else if (passive_status & PASSED_RIGHT) theta_candidate = (square_heading()*DEGS) - theta_offset;
			else if (hit_first == LEFT) theta_candidate = (square_heading()*DEGS) + theta_offset;
			else if (hit_first == RIGHT) theta_candidate = (square_heading()*DEGS) - theta_offset;
			// hit at the same time?
			else theta_candidate = (square_heading()*DEGS);

			// check how far away correction is from current theta
			if (abs(theta - theta_candidate) > THETA_CORRECT_LIMIT) {
				SERIAL_PRINT("PO");
				SERIAL_PRINTLN(theta_candidate*RADS);
				passive_status = PASSED_COOL_DOWN;
				return;
			}
			// else correct to candidate value
			theta = theta_candidate;
			SERIAL_PRINT('P');
			SERIAL_PRINTLN(theta_offset*RADS);

		}
		// suspicious of an intersection
		else {
			SERIAL_PRINT("PI");
			SERIAL_PRINT(correct_half_distance - correct_initial_distance);
			SERIAL_PRINT(' ');
			SERIAL_PRINTLN(current_distance() - correct_half_distance);
		}

		// reset even if not activated on this line (false alarm)
		passive_status = PASSED_COOL_DOWN;
	}

}

void passive_position_correct() {
	if (on_line(CENTER)) {
		++cycles_on_line;
		last_correct_distance = current_distance();
		// activate line following if close enough to target and is on a line
	}
	else {
		// false positive, not on line for enough cycles
		if (cycles_on_line < CYCLES_CROSSING_LINE && cycles_on_line >= 0) { cycles_on_line = 0; return; }
		else if (counted_lines >= LINES_PER_CORRECT && far_from_intersection(x, y)) {
			counted_lines = 0;
			// correct whichever one is closer to 0 or 200 
			// account for red line, can't detect along y so just correct to x
			if (abs(y - RENDEZVOUS_Y) < 0.5*GRID_WIDTH) {
				x = round(x / GRID_WIDTH) * GRID_WIDTH;
				// leaving line forward
				if (theta > -HALFPI && theta < HALFPI) x += HALF_LINE_WIDTH;
				else x -= HALF_LINE_WIDTH;
			}
			else {
				correct_to_grid();
			}
			SERIAL_PRINTLN('C');
		}
		else {
			// false alarm, cool down
			++counted_lines;
			SERIAL_PRINTLN('L');
			passive_status = PASSED_COOL_DOWN;
		}
		// either C or L will have last correct distance updated
		// last_correct_distance = current_distance();
		cycles_on_line = 0;
	}
}

void passive_red_line_correct() {
	// only correct to red line if not backing up
	if (abs(y - RENDEZVOUS_Y) < GRID_WIDTH*0.5 && layers[active_layer].speed > 0) {
		digitalWrite(bottom_led, HIGH);
		if (on_line(CENTER)) cycles_on_red_line = 0;
		else if (on_line(RED) && !on_line(CENTER)) {
			++cycles_on_red_line;
			// navigate trying to get back to red line and x is far enough forward
			if (seeking_red_line && RENDEZVOUS_X - x < 3*RENDEZVOUS_CLOSE) {
				waypoint(LAYER_NAV);
			}
		}
		else if (!on_line(RED)) {
			// not false alarm
			if (cycles_on_red_line >= CYCLES_CROSSING_LINE && current_distance() - last_correct_distance > DISTANCE_CENTER_TO_RED_ALLOWANCE) {
				SERIAL_PRINT("RC");
				SERIAL_PRINTLN(current_distance() - last_correct_distance);
				// direction and signs are taken care of by sin and cos
				float offset_y = DISTANCE_CENTER_TO_RED * sin(theta);
				// avoid correcting when parallel to horizontal line
				int offset_x = abs((int)x) % GRID_WIDTH;
				if (abs(offset_y) > NEED_TO_HOPPER_CORRECT && (offset_x < INTERSECTION_TOO_CLOSE*0.5 || offset_x > GRID_WIDTH - INTERSECTION_TOO_CLOSE*0.5) && parallel_to_horizontal()) {
					SERIAL_PRINTLN("-RC");
					cycles_on_red_line = 0;
					last_correct_distance = current_distance();
					return;
				}

				y = RENDEZVOUS_Y;
				// between [-180,0] left while going left
				// x += DISTANCE_CENTER_TO_RED * cos(theta);
				if (theta < 0) offset_y -= HALF_LINE_WIDTH;
				else offset_y += HALF_LINE_WIDTH;

				y += offset_y;

				last_red_line_distance = current_distance();
				last_correct_distance = last_red_line_distance;
				if (side_of_board == SIDE_RIGHT) side_of_board = SIDE_LEFT;
				else if (side_of_board == SIDE_LEFT) side_of_board = SIDE_RIGHT;
			}

			cycles_on_red_line = 0;
		}
	}
	else {
		digitalWrite(bottom_led, LOW);
		cycles_on_red_line = 0;
	}
}
void correct_to_hopper() {
	// extend theta backwards by the distance between the hopper and the center of the robot 
	float correct_x = boundaries[active_hopper].x - cos(theta)*BETWEEN_HOPPER_AND_CENTER;
	float correct_y = boundaries[active_hopper].y - sin(theta)*BETWEEN_HOPPER_AND_CENTER;
	if (abs(correct_x - x) > NEED_TO_HOPPER_CORRECT) {
		SERIAL_PRINTLN("HX");
		x = correct_x;
	}
	if (abs(correct_y - y) > NEED_TO_HOPPER_CORRECT) {
		SERIAL_PRINTLN("HY");
		y = correct_y;
	}
}

}	// end namespace