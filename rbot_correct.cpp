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

// control the correction layer
void user_correct() {
	// don't try to correct while running for your life
	if (layers[LAYER_BOUND].active || layers[LAYER_TURN].active || layers[LAYER_GET].active || layers[LAYER_PUT].active) return;
	
	if (layers[LAYER_COR].active) {

		if ((on_lines[LEFT] && on_lines[CENTER]) ||
			(on_lines[LEFT] && on_lines[RIGHT]) ||
			(on_lines[CENTER] && on_lines[RIGHT]) ||
			// or overturned
			(hit_first == LEFT && on_lines[RIGHT]) ||
			(hit_first == RIGHT && on_lines[LEFT])) {

			// correct both heading and position at this line crossing
			square_heading();
			correct_to_grid();
			counted_lines = 0;

			last_correct_distance = tot_distance;
			layers[LAYER_COR].active = false;

			Serial.println('Y');
		} 
	}
	// approaching from an angle, and have crossed enough lines; correct theta
	else if ((on_lines[LEFT] || on_lines[RIGHT]) && (tot_distance - last_correct_distance) > DISTANCE_PER_CORRECT) {
		byte offset_x = abs((int)x) % GRID_WIDTH;
		byte offset_y = abs((int)y) % GRID_WIDTH;
		// when only 1 position is close to a line but not both (meaning intersection)
		if ((offset_x < INTERSECTION_TOO_CLOSE || offset_x > GRID_WIDTH - INTERSECTION_TOO_CLOSE) ^
			(offset_y < INTERSECTION_TOO_CLOSE || offset_y > GRID_WIDTH - INTERSECTION_TOO_CLOSE)) {

			if (on_lines[LEFT])  hit_first = LEFT;
			else 				 hit_first = RIGHT;
			layers[LAYER_COR].active = true;
			// need to stop to turn
			hard_break();
			Serial.println('X');
		}
	}
}

}	// end namespace