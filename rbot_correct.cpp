#include "rbot.h"
#include "parameters.h"

namespace robot {

void correct_theta() {
	Layer& cor = layers[LAYER_COR];
	if (!cor.active) return;
	// if previously on break, resume drive
	if (paused) resume_drive();

	cor.speed = 0;

	// turn in place of the direction way from which one hit first
	if (hit_first == LEFT) cor.angle = -CORRECT_SPEED;
	else if (hit_first == RIGHT) cor.angle = CORRECT_SPEED;

}

// correct position using center sensor; assumes readings are up to date
void line_detect() {
	// don't line correct when turning in place
	if (layers[LAYER_TURN].active || layers[LAYER_COR].active || layers[LAYER_BOUND].active) return;

	// if enough distance passed for a passive theta correct
	if ((tot_distance - last_passive_correct_distance > DISTANCE_PER_PASSIVE_CORRECT)) {
		// first entering the lines
		passive_correct |= (on_lines & B111);	// right left center
		// passing the lines
		// passive_correct |= (~(passive_correct) ^ ~(on_lines)) << 3;
		if (passive_correct & LEFT) passive_correct |= ~(on_lines & LEFT) << 3;
		if (passive_correct & RIGHT) passive_correct |= ~(on_lines & RIGHT) << 3;
		if (passive_correct & CENTER) passive_correct |= ~(on_lines & CENTER) << 3;

		// one side passed and the other 
		if (passive_correct == )
	}



	if (on_line(CENTER)) {
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
				add_target(x, y, turn_to, TARGET_TURN);
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

void correct_against_line() {
	// don't try to correct while running for your life
	if (layers[LAYER_COR].active) {

		if ((on_line(LEFT) && on_line(CENTER)) ||
			(on_line(LEFT) && on_line(RIGHT)) ||
			(on_line(CENTER) && on_line(RIGHT)) ||
			// or overturned
			(hit_first == LEFT && on_line(RIGHT)) ||
			(hit_first == RIGHT && on_line(LEFT))) {

			// correct both heading and position at this line crossing
			square_heading();
			correct_to_grid();
			counted_lines = 0;
			if (paused) resume_drive();
			last_correct_distance = tot_distance;
			layers[LAYER_COR].active = false;
			// reset
			hit_first = CENTER;

			Serial.println('Y');
		} 
	}
	// don't start trying to correct if these layers are active
	else if (layers[LAYER_BOUND].active || layers[LAYER_TURN].active || layers[LAYER_GET].active || layers[LAYER_PUT].active) return;
	// actual turn in place, after being indicated that one side hit line
	else if (hit_first != CENTER && on_line(CENTER)) {
		layers[LAYER_COR].active = true;
		// need to stop to turn
		hard_break();
		Serial.println('X');
	}
	// approaching from an angle, and have crossed enough lines; correct theta
	else if ((on_line(LEFT) || on_line(RIGHT)) && (tot_distance - last_correct_distance) > DISTANCE_PER_CORRECT) {
		byte offset_x = abs((int)x) % GRID_WIDTH;
		byte offset_y = abs((int)y) % GRID_WIDTH;
		// when only 1 position is close to a line but not both (meaning intersection)
		if ((offset_x < INTERSECTION_TOO_CLOSE || offset_x > GRID_WIDTH - INTERSECTION_TOO_CLOSE) ^
			(offset_y < INTERSECTION_TOO_CLOSE || offset_y > GRID_WIDTH - INTERSECTION_TOO_CLOSE)) {

			if (on_line(LEFT))  hit_first = LEFT;
			else 				 hit_first = RIGHT;
		}
	}
}

}	// end namespace