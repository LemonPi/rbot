#include <Adafruit_TiCoServo.h>
#include "rbot.h"
#include "parameters.h"

namespace robot {

byte ball_status = BALL_LESS;
float get_initial_distance;
Adafruit_TiCoServo gate;
int cycles_on_line, counted_lines;
byte hit_first;

float last_correct_distance;

// called inside every go cycle
void user_behaviours() {
	correct_theta();
	get_ball();
}

// control the correction layer
void user_correct() {
	line_detect();
	correct_against_line();
}

// called after arriving
void user_waypoint() {
	// previous target was to get the ball (target+1 was immediately previous target)
	if (targets[target+1].type == TARGET_GET) {
		layers[LAYER_GET].active = true;
		// starting point of retrieving the robot
		get_initial_distance = tot_distance;
	}
	// don't have ball and the current target isn't to get to a ball
	else if (ball_status == BALL_LESS && targets[target].type != TARGET_GET) {
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
			// get the ball when you get there
			add_target(min_target.x, min_target.y, min_target.theta, TARGET_GET, true);

			Serial.println('b');
		}
	}
	// put ball away if has ball and at rendezvous point
	else if (targets[target+1].type == TARGET_PUT && ball_status == SECURED_BALL) {
		layers[LAYER_PUT].active = true;
	}
}


void initialize_rbot(byte servo_pin) {
	hit_first = CENTER;
	cycles_on_line = 0;	// counter for continuous cycles on line
	counted_lines = 0;
	gate.attach(servo_pin);
	open_gate();
}

void user_start() {
	cycles_on_line = 0;
	counted_lines = 0;
	last_correct_distance = 0;
}

}	// end namespace