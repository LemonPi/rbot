#include <Adafruit_TiCoServo.h>
#include "rbot.h"
#include "parameters.h"

namespace robot {

byte ball_status = BALL_LESS;
bool getting_ball = false;
float get_initial_distance;
Adafruit_TiCoServo gate;

// called inside every go cycle
void user_behaviours() {
	correct_theta();
	get_ball();
}

// called after arriving
void user_waypoint() {
	// arrived at a hopper (getting ball at cur_target) 
	if (getting_ball) {
		layers[LAYER_GET].active = true;
		// starting point of retrieving the robot
		get_initial_distance = tot_distance;
	}
	else if (ball_status == BALL_LESS) {
		// find closest hopper to retrieve
		float min_distance = 10000;
		float distance;
		Target min_target;
		Target cur_target;
		for (byte h = 0; hoppers[h] < boundary_num && h < HOPPER_NUM; ++h) {
			cur_target = approach_hopper(hoppers[h]);
			distance = sqrt(sq(x - cur_target.x) + sq(y - cur_target.y));
			if (distance < min_distance) {
				min_distance = distance; 
				min_target = cur_target;
			}
		}
		// go to that hopper if one exists
		if (min_distance != 10000) {
			add_target(min_target.x, min_target.y, min_target.theta, true);

			// get the ball when you get there
			getting_ball = true;
		}
	}
	// put ball away if has ball and at rendezvous point
	else if (ball_status == SECURED_BALL && sqrt(sq(x - RENDEZVOUS_X) + sq(y - RENDEZVOUS_Y)) < RENDEZVOUS_CLOSE) {
		layers[LAYER_PUT].active = true;
	}
}


void initialize_rbot(byte servo_pin) {
	gate.attach(servo_pin);
	open_gate();
}


}	// end namespace