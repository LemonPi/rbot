#include <rbot.h>
#include <robot.h>
#include "parameters.h"

namespace robot {

byte hoppers[HOPPER_NUM] = {HOPPER1, HOPPER2, HOPPER3, HOPPER4};

const int x_lookup[4] = {1230,1010,790,570};
const int y_lookup[7] = {420,550,670,800,930,1050,1170};

// get ball behaviour, should only occur at hopper approach
void get_ball() {
	Layer& get = layers[LAYER_GET];
	if (!get.active) return;

	// either going forward or backward, always no angle
	get.angle = 0;
	// go forward until ball is detected or arbitrary additional distance travelled
	if (ball_status == BALL_LESS) {
		if (tot_distance - get_initial_distance < GET_DISTANCE) get.speed = GET_SPEED;
		else {
			// then close servo gate
			close_gate();
			hard_break();
			ball_status = JUST_GOT_BALL;
		}
	}
	// after securing ball, drive backwards for the same amount of distance
	else if (ball_status == SECURED_BALL) {
		if (paused) resume_drive();
		if (tot_distance - get_initial_distance < 3.5*GET_DISTANCE) get.speed = -GET_SPEED;
		// after getting ball, return to rendezvous point
		else {
			layers[LAYER_GET].active = false;
			add_target(RENDEZVOUS_X, RENDEZVOUS_Y, 0, TARGET_PUT, true);
			close_hoppers();
		}
	}
	// in the middle of closing the gate, wait a couple cycles
	else {
		++ball_status;
	}
}

// hopper is defined by 3 pillar index
void add_hopper(byte p1, byte p2, byte p3) {
	int px[3] = {x_lookup[p1 / 7], x_lookup[p2 / 7], x_lookup[p3 / 7]};
	int py[3] = {y_lookup[p1 % 7], y_lookup[p2 % 7], y_lookup[p3 % 7]};

	for (int i = 0; i < 3; ++i) {
		add_boundary(px[i], py[i], PILLAR_RADIUS);
	}
	// pillar is a boundary
	add_boundary((px[0]+px[1]+px[2])/3, (py[0]+py[1]+py[2])/3, HOPPER_RADIUS);
}

Target approach_hopper(byte hopper) {
	double max_dist = 0; 
	int max_index = 0;
	// corner hoppers have only 1 direction of approach
	if (hopper == HOPPER3 || hopper == HOPPER4) {
		// find center point of the 2 pillars
		double mid_point_x = (boundaries[hopper-1].x + boundaries[hopper-2].x)/2;
		double mid_point_y = (boundaries[hopper-1].y + boundaries[hopper-2].y)/2;
		// from center point to the hopper
		double off_x = boundaries[hopper].x - mid_point_x;
		double off_y = boundaries[hopper].y - mid_point_y;

		return Target{mid_point_x - off_x, mid_point_y - off_y, atan2(off_y, off_x)};
	}
	// test 3 pillars with boundary indices before the hopper index
	else {
		// find furthest away pillar
		for (int i = 1; i < 4; ++i) {
			double dist; 
			// avoid boundary hasn't updated yet
			if (boundaries[hopper-i].distance == 0) dist = sqrt(sq(boundaries[hopper-i].x - x) + sq(boundaries[hopper-i].y - y));
			else dist = boundaries[hopper-i].distance;

			if (dist > max_dist) {max_dist = dist; max_index = hopper-i;}
		}

		// offset vector from hopper
		double off_x = boundaries[max_index].x - boundaries[hopper].x;
		double off_y = boundaries[max_index].y - boundaries[hopper].y;

		return Target{boundaries[hopper].x - off_x, boundaries[hopper].y - off_y, atan2(off_y, off_x)};
	}
}

void open_hoppers() {
	boundaries[HOPPER1].r = HOPPER_RADIUS;
	boundaries[HOPPER2].r = HOPPER_RADIUS;
	boundaries[HOPPER3].r = HOPPER_RADIUS;
	boundaries[HOPPER4].r = HOPPER_RADIUS;
}

void close_hoppers() {
	boundaries[HOPPER1].r = COLONY_RADIUS;
	boundaries[HOPPER2].r = COLONY_RADIUS;
	boundaries[HOPPER3].r = COLONY_RADIUS;
	boundaries[HOPPER4].r = COLONY_RADIUS;
}

}	// end namespace