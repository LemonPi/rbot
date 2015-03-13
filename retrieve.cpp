#include <rbot.h>
#include <robot.h>
#include "parameters.h"

namespace robot {

const int x_lookup[4] = {1230,1010,790,570};
const int y_lookup[7] = {420,550,670,800,930,1050,1170};

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

void approach_hopper(byte hopper) {
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

		add_target(mid_point_x - off_x, mid_point_y - off_y, atan2(off_y, off_x), true);
	}
	// test 3 pillars with boundary indices before the hopper index
	else {
		// find furthest away pillar
		for (int i = 1; i < 4; ++i) {
			double dist = boundaries[hopper-i].distance;
			if (dist > max_dist) {max_dist = dist; max_index = hopper-i;}
		}

		// offset vector from hopper
		double off_x = boundaries[max_index].x - boundaries[hopper].x;
		double off_y = boundaries[max_index].y - boundaries[hopper].y;

		add_target(boundaries[hopper].x - off_x, boundaries[hopper].y - off_y, atan2(off_y, off_x), true);		
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