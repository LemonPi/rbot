#include "rbot.h"
#include "parameters.h"

namespace robot {

void correct_theta() {
	Layer& cor = layers[LAYER_COR];
	if (!cor.active) return;
	// if previously on break, resume drive
	if (paused) resume_drive();

	cor.speed = CORRECT_SPEED;
	// left is ahead of right
	if (on_line(LEFT)) cor.angle = -CORRECT_SPEED;
	else if (on_line(RIGHT)) cor.angle = CORRECT_SPEED;
	else {
		square_heading();
		last_target_distance = tot_distance;
		cor.active = false; 
	}

}

// control the correction layer
void user_correct() {
	if (layers[LAYER_COR].active) {
		// stop when any 2 is on line
		if (on_line(LEFT) && on_line(CENTER) ||
			on_line(LEFT) && on_line(RIGHT) ||
			on_line(CENTER) && on_line(RIGHT)) {

			square_heading();
			last_target_distance = tot_distance;
			layers[LAYER_COR].active = false;
		} 
	}
	// approaching from an angle, and have crossed enough lines; correct theta
	else if ((on_line(LEFT) || on_line(RIGHT)) && (tot_distance - last_correct_distance) > DISTANCE_PER_CORRECT) {
		byte offset_x = abs((int)x) % GRID_WIDTH;
		byte offset_y = abs((int)y) % GRID_WIDTH;
		// not too close to intersection as that wouldn't represent rotating to a straight line
		if (INTERSECTION_TOO_CLOSE < offset_x && offset_x < GRID_WIDTH - INTERSECTION_TOO_CLOSE &&
			INTERSECTION_TOO_CLOSE < offset_y && offset_y < GRID_WIDTH - INTERSECTION_TOO_CLOSE) {

			layers[LAYER_COR].active = true;
			// need to stop to turn
			hard_break();
			Serial.println('X');
		}
	}
}

}	// end namespace