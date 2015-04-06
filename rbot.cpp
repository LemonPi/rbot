#include <Adafruit_TiCoServo.h>
#include "rbot.h"
#include "parameters.h"

namespace robot {

byte ball_pin;
byte bottom_led;

byte ball_status = BALL_LESS;
float get_initial_distance;
Adafruit_TiCoServo gate;
int cycles_on_line, counted_lines;
int cycles_on_red_line;
byte active_hopper;
int available_hoppers;
byte hit_first;

bool corrected_while_backing_up;

int passive_status;
float correct_initial_distance;
float correct_half_distance;
float last_correct_distance;

byte turned_to_put;

// called inside every go cycle
void user_behaviours() {
	get_ball();
	put_ball();
	if (active_layer == LAYER_TURN) {
		SERIAL_PRINT(layers[LAYER_TURN].speed);
		SERIAL_PRINT('|');
		SERIAL_PRINTLN(layers[LAYER_TURN].angle);
	}
}

// control the correction layer
void user_correct() {
	if (layers[LAYER_TURN].active) return;
	passive_correct();
	passive_position_correct();
	passive_red_line_correct();
}

// called after arriving
void user_waypoint() {
	// previous target was to get the ball (target+1 was immediately previous target)
	if (targets[target+1].type == TARGET_GET) {
		layers[LAYER_GET].active = true;
		// starting point of retrieving the robot
		get_initial_distance = tot_distance;
	}
	// previous target was to put the ball
	else if (targets[target+1].type == TARGET_PUT && ball_status == SECURED_BALL) {
		layers[LAYER_PUT].active = true;
	}
	// don't have ball and the current target isn't to get to a ball
	else if (ball_status == BALL_LESS && targets[target].type != TARGET_GET && target == NONE_ACTIVE) {
		// find closest hopper to retrieve
		float min_distance = 10000;
		float distance;
		Target min_target;
		Target cur_target;
		// active_hopper is the hopper number, at the end of the loop will be the one to select
		byte selected_hopper;
		// load of hopper needs to be > 0 for it to be considered
		for (byte h = 0; hoppers[h].index < boundary_num && h < HOPPER_NUM && hoppers[h].load > 0; ++h) {
			// alternate between closest 2 hoppers
			if (hoppers[h].index == active_hopper && available_hoppers > 1) continue;

			cur_target = approach_hopper(hoppers[h].index);
			distance = sqrt(sq(x - cur_target.x) + sq(y - cur_target.y));
			// find a close hopper
			// also check that getting there won't bring you too close to another hopper
			if (distance < min_distance) {
				min_distance = distance; 
				min_target = cur_target;
				selected_hopper = h;
			}
		}
		// go to that hopper if one exists
		if (min_distance != 10000) {
			// get the ball when you get there
			add_target(min_target.x, min_target.y, min_target.theta, TARGET_GET, true);
			// anticipate decreasing the selected hopper's load (can't easily do that at the point of getting)
			--hoppers[selected_hopper].load;
			if (hoppers[selected_hopper].load == 0) --available_hoppers;
			active_hopper = hoppers[selected_hopper].index;
			SERIAL_PRINTLN('g');
		}
	}
}


void initialize_rbot(byte servo_pin, byte ball_proximity_pin, byte bot_led) {
	ball_pin = ball_proximity_pin;
	pinMode(ball_pin, INPUT);
	bottom_led = bot_led;
	pinMode(bot_led, OUTPUT);
	
	cycles_on_line = 0;	// counter for continuous cycles on line
	cycles_on_red_line = 0;
	counted_lines = 0;
	passive_status = PASSED_NONE;
	ball_status = BALL_LESS;
	active_hopper = 0;
	available_hoppers = 0;
	corrected_while_backing_up = false;
	last_correct_distance = 0;
	turned_to_put = 0;
	gate.attach(servo_pin);
	open_gate();
}

void user_start() {
	cycles_on_line = 0;
	cycles_on_red_line = 0;
	counted_lines = 0;
	corrected_while_backing_up = false;
	passive_status = PASSED_NONE;
}

void user_stop() {
}

}	// end namespace