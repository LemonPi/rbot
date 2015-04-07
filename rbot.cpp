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
float last_red_line_distance;
byte side_of_board;
bool seeking_red_line;

byte turned_to_put;

// called inside every go cycle
void user_behaviours() {
	// going to rendezvous point and thinking that I'm nearly there, but haven't touched a red line yet
	if (active_layer == LAYER_NAV && 
		targets[target].type == TARGET_PUT && abs(y - RENDEZVOUS_Y) < RENDEZVOUS_CLOSE && 
		(!on_line(RED) || (on_line(RED) && on_line(CENTER))) &&	// both on line means black line, disregard
	 	((current_distance() - last_red_line_distance) > 5*RENDEZVOUS_CLOSE || last_red_line_distance == 0)) {
		if (side_of_board == SIDE_RIGHT) y += 2*RENDEZVOUS_CLOSE;
		else if (side_of_board == SIDE_LEFT) y -= 2*RENDEZVOUS_CLOSE;
		SERIAL_PRINTLN("RN");

		// in case navigation gets greedy and releases control to turn in place
		layers[LAYER_TURN].active = false;
		// miles to go before robot is at RR
		layers[LAYER_NAV].active = true;
		seeking_red_line = true;
		++process_cycles;
	}
	get_ball();
	put_ball();
	if (active_layer == LAYER_NAV) {
		SERIAL_PRINT(layers[active_layer].speed);
		SERIAL_PRINT('|');
		SERIAL_PRINTLN(layers[active_layer].angle);
		
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
	if (targets[target+1].type == TARGET_PUT && seeking_red_line) {
		seeking_red_line = false;
		// emergency stop by red line correct
		SERIAL_PRINTLN("RS");
		// add a turn in place
		++target;
		layers[LAYER_TURN].active = true;
		layers[LAYER_NAV].active = false;
		// add_target(x, y, 0, TARGET_PUT);
	}
	else if (targets[target+1].type == TARGET_GET) {
		layers[LAYER_GET].active = true;
		// starting point of retrieving the robot
		get_initial_distance = tot_distance;
	}
	// previous target was to put the ball
	else if (targets[target+1].type == TARGET_PUT && ball_status == SECURED_BALL) {
			layers[LAYER_PUT].active = true;
			SERIAL_PRINT("RR");
			SERIAL_PRINTLN((current_distance() - last_red_line_distance));

			y = RENDEZVOUS_Y;
	}
	// don't have ball and the current target isn't to get to a ball
	else if (ball_status == BALL_LESS && targets[target].type != TARGET_GET && target == NONE_ACTIVE) {
		// find closest hopper to retrieve
		// active_hopper is the hopper number, at the end of the loop will be the one to select
		byte selected_hopper;
		// load of hopper needs to be > 0 for it to be considered
		for (byte h = 0; hoppers[h].index < boundary_num && h < HOPPER_NUM && hoppers[h].load > 0; ++h) {
			// alternate between closest 2 hoppers
			if (hoppers[h].index == active_hopper && available_hoppers > 1) continue;

			follow_hopper_waypoints(h);
			selected_hopper = h;
			break;			

		}
		--hoppers[selected_hopper].load;
		if (hoppers[selected_hopper].load == 0) --available_hoppers;
		active_hopper = hoppers[selected_hopper].index;
		SERIAL_PRINTLN('g');
		
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
	last_red_line_distance = 0;
	side_of_board = SIDE_RIGHT;
	seeking_red_line = false;
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