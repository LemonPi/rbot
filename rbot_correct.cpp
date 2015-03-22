#include "rbot.h"
#include "parameters.h"

namespace robot {

void correct_theta() {
	Layer& cor = layers[LAYER_COR];
	if (!cor.active) return;
	// control speed based on center status
	switch (center_status) {
		case INFRONT: cor.speed = CORRECT_SPEED; break;
		case BEHIND: cor.speed = -CORRECT_SPEED; break;
		case ONTOP: cor.speed = 0;
	}
	// center on top
	// if previously on break, resume drive
	if (paused) resume_drive();

	// turn in place of the direction way from which one hit first
	if (hit_first == LEFT) cor.angle = -CORRECT_SPEED;
	else if (hit_first == RIGHT) cor.angle = CORRECT_SPEED;

	// reverse the angle if center is behind
	if (center_status == BEHIND) cor.angle = -cor.angle;
}


// correct position using center sensor; assumes readings are up to date
void line_detect() {
	// don't line correct when turning in place
	if (layers[LAYER_TURN].active || layers[LAYER_COR].active || layers[LAYER_BOUND].active) return;


	if (on_line(CENTER)) {
		++cycles_on_line;
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

// choose the closest line segment to correct against
void active_correct_choose_line() {
	// choose a line segment to correct against
	if (!layers[LAYER_COR].active && tot_distance - last_correct_distance > DISTANCE_PER_CORRECT) {
		// choose perpendicular line based on theta
		int closest_normal = square_heading();
		int candidate_x, candidate_y;
		if (closest_normal == DIR_UP) {
			// offset from the closest line
			int offset_x = GRID_WIDTH - (abs((int)x) % GRID_WIDTH);
			int offset_y = offset_x * tan(theta);

			candidate_x = x + offset_x;
			candidate_y = y + offset_y;
			// check to see if candidate is far enough away from 
			while (abs(offset_y) < INTERSECTION_TOO_CLOSE || abs(offset_y) > GRID_WIDTH - INTERSECTION_TOO_CLOSE) {
				candidate_x += GRID_WIDTH;
				candidate_y += GRID_WIDTH * tan(theta);
				offset_y = candidate_y % GRID_WIDTH;
			}
		}
		else if (closest_normal == DIR_RIGHT) {
			int offset_y = GRID_WIDTH - (abs((int)y) % GRID_WIDTH);
			int offset_x = offset_y / tan(theta);

			candidate_x = x + offset_x;
			candidate_y = y + offset_y;
			// check to see if the chosen y will be far away from an intersection
			while (abs(offset_x) < INTERSECTION_TOO_CLOSE || abs(offset_x) > GRID_WIDTH - INTERSECTION_TOO_CLOSE) {
				candidate_y += GRID_WIDTH;
				candidate_x += GRID_WIDTH / tan(theta);
				offset_x = candidate_x % GRID_WIDTH;
			}
		}
		else if (closest_normal == DIR_LEFT) {
			int offset_y = - (abs((int)y) % GRID_WIDTH);
			// offset_y will be negative, tan(theta) will also be negative if > -HALFPI, positive if < -HALFPI
			int offset_x = offset_y / tan(theta);
			candidate_x = x + offset_x;
			candidate_y = y + offset_y;
			// check to see if the chosen y will be far away from an intersection
			while (abs(offset_x) < INTERSECTION_TOO_CLOSE || abs(offset_x) > GRID_WIDTH - INTERSECTION_TOO_CLOSE) {
				candidate_y -= GRID_WIDTH;
				candidate_x -= GRID_WIDTH / tan(theta);
				offset_x = candidate_x % GRID_WIDTH;
			}
		}
		else {
			int offset_x = - (abs((int)x) % GRID_WIDTH);
			int offset_y = offset_x * tan(theta);

			candidate_x = x + offset_x;
			candidate_y = y + offset_y;
			// check to see if the chosen y will be far away from an intersection
			while (abs(offset_y) < INTERSECTION_TOO_CLOSE || abs(offset_y) > GRID_WIDTH - INTERSECTION_TOO_CLOSE) {
				candidate_x -= GRID_WIDTH;
				candidate_y -= GRID_WIDTH * tan(theta);
				offset_y = candidate_y % GRID_WIDTH;
			}
		}
		add_target(candidate_x, candidate_y, ANY_THETA, TARGET_COR);
		layers[LAYER_COR].active = true;
		hit_first = NONE_ACTIVE;
		center_status = INFRONT;
		left_crossed = right_crossed = 0;
	}
}

bool far_from_intersection(int candidate_x, int candidate_y) {
	byte ox = abs((int)candidate_x) % GRID_WIDTH;
	byte oy = abs((int)candidate_y) % GRID_WIDTH;
	return (ox < INTERSECTION_TOO_CLOSE || ox > GRID_WIDTH - INTERSECTION_TOO_CLOSE) ^
			(oy < INTERSECTION_TOO_CLOSE || oy > GRID_WIDTH - INTERSECTION_TOO_CLOSE);
}

// listens to line passes when trying to actively correct and when you are close enough
// gurantee that inside here, target is TARGET_COR
void active_correct_against_line() {
	if (layers[LAYER_BOUND].active) return;
	if (layers[LAYER_COR].active && target_distance < CORRECT_CLOSE_ENOUGH && targets[target].type == TARGET_COR) {

		// stop when both are on the line
		if (on_line(LEFT) && on_line(RIGHT)) {	

			// correct both heading and position at this line crossing
			Serial.println(on_lines);
			theta = square_heading() * DEGS;
			x = targets[target].x;
			y = targets[target].y;
			// counted_lines = 0;
			if (paused) resume_drive();
			last_correct_distance = tot_distance;
			layers[LAYER_COR].active = false;

			Serial.println('Y');
		} 
	}
	// approaching from an angle, and have crossed enough lines; correct theta
	else if (on_lines[CENTER] && (tot_distance - last_correct_distance) > DISTANCE_PER_CORRECT) {
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
			// reset
			// return to normal navigation
			// byte temp = on_lines & B111;
			Serial.print('Y');
			waypoint();
		} 

		// haven't hit a line yet with any sensors		
		if (on_line(LEFT))  	 hit_first = LEFT;
		else if (on_line(RIGHT)) hit_first = RIGHT;
		

		// update where the center is relative to the robot
		if (on_line(CENTER) && center_status != ONTOP)) {
			// need to stop to turn
			center_status = ONTOP;
			// hard_break();
			Serial.println('X');
		}
		// update for whether it left the line while going forward
		else if (center_status == ONTOP) {
			// assume a speed of 0 will carry the robot in front of the line
			if (layers[LAYER_COR].speed >= 0) center_status = BEHIND;
			else center_status = INFRONT;
		}

	}
}

}	// end namespace