#include <rbot.h>
#include <robot.h>
#include "parameters.h"

namespace robot {

Hopper hoppers[HOPPER_NUM] = {{HOPPER1, 7, 0}, {HOPPER2, 7, 0}, {HOPPER3, 4, 0}, {HOPPER4, 4, 0}};
Target hopper_waypoints[HOPPER_NUM][5];

const int x_lookup[4] = {1230,1010,790,570};
const int y_lookup[7] = {420,550,670,800,930,1050,1170};

// get ball behaviour, should only occur at hopper approach
void get_ball() {
	Layer& get = layers[LAYER_GET];
	if (!get.active) return;

	if (LAYER_GET == active_layer) {
		SERIAL_PRINT(layers[LAYER_GET].speed);
		SERIAL_PRINT('|');
		SERIAL_PRINTLN(layers[LAYER_GET].angle);
	}

	// either going forward or backward, always no angle
	// go forward until ball is detected or arbitrary additional distance travelled
	if (ball_status < CAUGHT_BALL) {
		if (caught_ball()) ++ball_status;

		get.speed = GET_SPEED;
		if (abs(boundaries[active_hopper].theta) < THETA_TOLERANCE) get.angle = 0;
		// turn slightly to face hopper
		else if (boundaries[active_hopper].theta < 0) get.angle = -GET_TURN;
		else get.angle = GET_TURN;

		if (ball_status == CAUGHT_BALL) {
			// got to the ball, can also correct for position to be near hopper
			correct_to_hopper();
			// then close servo gate
			close_gate();
			hard_break(LAYER_GET);
			get_initial_distance = tot_distance;
		}
	}
	// after securing ball, drive backwards 
	else if (ball_status == SECURED_BALL) {
		// initial kick to go straight
		static int kick_cycle = 0;
		if (get.speed > -GET_SPEED) {get.angle = 13; }
		else if (get.angle > 0) {
			if (kick_cycle <= 0) {
				if (get.angle < 7) kick_cycle = 7 - get.angle;
				--get.angle;
			}
			else --kick_cycle;
		}
	
		get.speed = -GET_SPEED;
		// get.angle = 0;
		if (paused) resume_drive(LAYER_GET);
		// back up until you hit a line to correct position
		if (on_line(CENTER)) {
			corrected_while_backing_up = true; 
			correct_to_grid();
			SERIAL_PRINTLN("CWB");
		}
		
		// backed up far enough
		if (tot_distance - get_initial_distance > 5*GET_DISTANCE) {
			// corrected_while_backing_up && 
			// after getting ball, return to rendezvous point
			corrected_while_backing_up = false;
			layers[LAYER_GET].active = false;
			add_target(RENDEZVOUS_X, RENDEZVOUS_Y, 0, TARGET_PUT);
			close_hoppers();
			// hard_break(LAYER_GET, 3);
		}
	}
	// in the middle of closing the gate, wait a couple cycles
	else {
		++ball_status;
	}
}

// hopper is defined by 3 pillar index
void add_hopper(byte p1, byte p2, byte p3, byte load) {
	int px[3] = {x_lookup[p1 / 7], x_lookup[p2 / 7], x_lookup[p3 / 7]};
	int py[3] = {y_lookup[p1 % 7], y_lookup[p2 % 7], y_lookup[p3 % 7]};

	for (int i = 0; i < 3; ++i) {
		add_boundary(px[i], py[i], PILLAR_RADIUS);
	}

	// manually set load
	if (load != DEFAULT_LOAD) {
		switch (boundary_num) {
			case HOPPER1: hoppers[0].load = load; break;
			case HOPPER2: hoppers[1].load = load; break;
			case HOPPER3: hoppers[2].load = load; break;
			case HOPPER4: hoppers[3].load = load; break;
			default: break;
		}
	}
	if (load != 0) ++available_hoppers;
	// pillar is a boundary
	add_boundary((px[0]+px[1]+px[2])/3, (py[0]+py[1]+py[2])/3, HOPPER_RADIUS);
}
void add_hopper_waypoint(int h, int t_x, int t_y) {
	Hopper& hopper = hoppers[h];
	Target& h_waypoint = hopper_waypoints[h][hopper.waypoint];
	h_waypoint.x = t_x;
	h_waypoint.y = t_y;
	h_waypoint.type = TARGET_NAV;
	++hopper.waypoint;
}
void last_hopper_waypoint(int h) {
	Hopper& hopper = hoppers[h];
	Target& last_waypoint = hopper_waypoints[h][0];
	int hx = boundaries[hopper.index].x;
	int hy = boundaries[hopper.index].y;
	last_waypoint.theta = atan2(hy - last_waypoint.y, hx - last_waypoint.x);
	SERIAL_PRINT("last waypoint: ");
	SERIAL_PRINT(hopper_waypoints[h][0].x);
	SERIAL_PRINT(' ');
	SERIAL_PRINT(hopper_waypoints[h][0].y);
	SERIAL_PRINT(' ');
	SERIAL_PRINTLN(hopper_waypoints[h][0].theta * RADS);
}
void follow_hopper_waypoints(byte h) {
	Hopper& hopper = hoppers[h];
	if (hopper.waypoint == 0) {
		SERIAL_PRINT("No waypoints:");
		SERIAL_PRINTLN(h);
		return;
	}
	
	add_target(hopper_waypoints[h][0].x, hopper_waypoints[h][0].y, hopper_waypoints[h][0].theta, TARGET_GET, true);

	for (byte w = 1; w < hopper.waypoint; ++w) {
		add_target(hopper_waypoints[h][w].x, hopper_waypoints[h][w].y, ANY_THETA);
	}
}

void add_corner_hoppers() {
	add_boundary(24,200,PILLAR_RADIUS);
	add_boundary(200,24,PILLAR_RADIUS);
	add_boundary(60,60,HOPPER_RADIUS);

	add_boundary(24,1400,PILLAR_RADIUS);
	add_boundary(1576,24,PILLAR_RADIUS);
	add_boundary(60,1540,HOPPER_RADIUS);
	available_hoppers += 2;
}

bool caught_ball() {
	return digitalRead(ball_pin);
}

// propose a target for approaching the hopper
Target approach_hopper(byte hopper) {
	// corner hoppers have only 1 direction of approach
	if (hopper == HOPPER3 || hopper == HOPPER4) {
		// find center point of the 2 pillars
		double mid_point_x = (boundaries[hopper-1].x + boundaries[hopper-2].x)/2;
		double mid_point_y = (boundaries[hopper-1].y + boundaries[hopper-2].y)/2;
		// from center point to the hopper
		double off_x = APPROACH_SCALAR * boundaries[hopper].x - mid_point_x;
		double off_y = APPROACH_SCALAR * boundaries[hopper].y - mid_point_y;

		return Target{mid_point_x - off_x, mid_point_y - off_y, atan2(off_y, off_x)};
	}
	// test 3 pillars with boundary indices before the hopper index
	else {
		// assume target safe by default
		bool safe_target = false;
		byte exclude_pillar = 200;	// don't exclude any pillars initially

		double off_x, off_y, candidate_x, candidate_y;
		// might have to stretch offset if target is too close to wall
		float approach_stretch_factor = 1.1;

		while (!safe_target) {
			safe_target = true;
			byte max_index = hopper_select(hopper, exclude_pillar);
			// offset vector from hopper
			off_x = APPROACH_SCALAR * (boundaries[max_index].x - boundaries[hopper].x);
			off_y = APPROACH_SCALAR * (boundaries[max_index].y - boundaries[hopper].y);

			candidate_x = boundaries[hopper].x - off_x;
			candidate_y = boundaries[hopper].y - off_y;
			
			// check if candidate x and y are too close to another hopper
			for (byte h = 0; hoppers[h].index < boundary_num && h < HOPPER_NUM; ++h) {
				// only consider other hoppers
				if (hoppers[h].index == hopper) continue;
				float turning_room = sqrt(sq(boundaries[hoppers[h].index].x - candidate_x) + sq(boundaries[hoppers[h].index].y - candidate_y));
				
				// reconsider if distance is too close
				if (turning_room < OTHER_HOPPER_TOO_CLOSE) {
					exclude_pillar = max_index;
					safe_target = false;
				}
			}

		} 

		return Target{candidate_x, candidate_y, atan2(off_y, off_x)};
	}
}

byte hopper_select(byte hopper, byte exclude) {
	double max_dist = 0;
	byte max_index = 0;
	// find furthest away pillar
	for (byte i = 1; i < 4; ++i) {
		// exclude the target produced by this pillar (since it'd bring the robot too close to another hopper)
		if (hopper - i == exclude) continue;

		double dist; 
		// avoid boundary hasn't updated yet
		if (boundaries[hopper-i].distance == 0) dist = sqrt(sq(boundaries[hopper-i].x - x) + sq(boundaries[hopper-i].y - y));
		else dist = boundaries[hopper-i].distance;

		if (dist > max_dist) {max_dist = dist; max_index = hopper-i;}
	}
	return max_index;
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