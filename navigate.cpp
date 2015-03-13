#include <Arduino.h>
#include <hbridge.h>
#include <robot.h>
#include "parameters.h"

namespace robot {

// set target from origin (last reset point)
int add_target(double tx, double ty, double td, bool rad) {
	if (target + 1 >= TARGET_MAX) return -1;

	if (td != ANY_THETA) {
		// square turns at intersections, so in hard turn can use on-line to tell if turned enough 
		if ((int)td % 90 == 0 && 
			((int)tx % 200 == 0 || (int)ty % 200 == 0)) 
			square_turn = true;

		if (!rad) td *= PI / 180.0;
	}
	++target;
	targets[target].x = tx;
	targets[target].y = ty;
	targets[target].theta = td;
	layers[LAYER_NAV].active = true;
	return target;
}

// turning in place
// check if turn has been completed
void hard_turn() {
	Layer& turn = layers[LAYER_TURN];
	
	// if (turn.active) Serial.println('t');
	// else Serial.println('n');	
	turn.speed = 0;				// turn in place, no translational velocity
	if (!turn.active || target == NONE_ACTIVE) { turn.angle = 0; return; }

	double to_turn = targets[target].theta - theta;
	// would be faster to turn in the opposite direction 
	if (to_turn > PI) to_turn = TWOPI - to_turn;
	else if (to_turn < -PI) to_turn = TWOPI + to_turn;


	// square turns can rely on lines to align angles, center has to be on a line
	if (sensor_num == SENSOR_MAX && square_turn && on_line(CENTER)) {
		// completed turn when sides are also on line
		if (on_line(LEFT) && on_line(RIGHT)) {
			// fix theta to be the closest HALFPI / theta (set by target_theta)
			theta = targets[target].theta;
			Serial.println("lt");
			waypoint();
			return;
		}
	}
	// turn until theta ~= target_theta
	else if (abs(to_turn) < THETA_TOLERANCE) {
		Serial.println("dt");
		waypoint();
		return;
	}
	
	// turn with speed based on target_theta - theta
	turn.angle = to_turn * NAV_TURN * 4;
	// clamp to between min speed and top speed
	if (abs(turn.angle) < MIN_SPEED) {
		if (turn.angle < 0) turn.angle = -MIN_SPEED;
		else turn.angle = MIN_SPEED;
	}
	else if (abs(turn.angle) > TOP_SPEED) {
		if (turn.angle < 0) turn.angle = -TOP_SPEED;
		else turn.angle = TOP_SPEED;
	}

}

// get to a target waypoint by 
// resets target_x, target_y when arrived
void navigate() {
	Layer& nav = layers[LAYER_NAV];
	if (!nav.active) return;
	locate_target();	// calculate target_distance and target_theta

	// turn in place
	// not too close to active boundary
	if ((active_boundary == NONE_ACTIVE || 	// no active boundaries
			(boundaries[active_boundary].distance > target_distance &&	// target is closer than the boundary
			abs(boundaries[active_boundary].theta - heading_error) > 0.3)) &&
		target_distance > TARGET_IMMEDIATE &&
		!layers[LAYER_TURN].active &&	// not already turning 
		abs(heading_error) > CAN_TURN_IN_PLACE) { 	// need large enough of a turn)

		// push temporary targets (stationary, but turning)
		add_target(x, y, heading_error + theta, true);
		layers[LAYER_TURN].active = true;

		// update target_distance to not be 0
		// target_distance = TURNING_IN_PLACE;
		// Serial.println(active_boundary);
		Serial.print("t ");
		Serial.println(heading_error + theta);
		
	}

	// arrived at target (can't get any closer presumably)
	else if (((target_distance < TARGET_IMMEDIATE) ||
			 ((target_distance < TARGET_CIRCLE) && (target_distance > last_target_distance)))) {

		Serial.println('c');
		nav.active = false;	// no longer need to navigate	

		if (abs(targets[target].theta - ANY_THETA) > 1 && 	// target isn't just any theta
			abs(targets[target].theta - theta) > THETA_TOLERANCE) {	// still needs turning
			layers[LAYER_TURN].active = true;
		}
		// don't need to turn anymore
		else {
			Serial.println('f');
			waypoint();
		}
	}
	// still seeking target
	else {
		nav.active = true;
		// steer heading toward target
		if (abs(heading_error) < THETA_TOLERANCE) nav.angle = 0;
		else {
			if (heading_error < 0) nav.angle = -NAV_TURN;
			else nav.angle = NAV_TURN;
		}
		// slow down closer to target
		if (target_distance < TARGET_CLOSE) 
			nav.speed = TOP_SPEED * (target_distance/TARGET_CLOSE);
		else nav.speed = TOP_SPEED;
		// prevent going backwards/stalling when navigation
		if (nav.speed < NAV_TURN) nav.speed = NAV_TURN;
	}
	last_target_distance = target_distance;
}

// updates target_distance and heading_error
void locate_target() {
	if (target < 0) return;
	double diff_x = targets[target].x - x;
	double diff_y = targets[target].y - y;
	double diff_theta = atan2(diff_y, diff_x);
	target_distance = sqrt(sq(diff_x) + sq(diff_y));

	// keep heading error within [-PI, PI]
	heading_error = diff_theta - theta;
	if (heading_error > PI) heading_error -= TWOPI;
	else if (heading_error < -PI) heading_error += TWOPI;
}

// arrived at target, called at the end of other behaviours
// decides what behaviours to enable based on game and current state
void waypoint() {
	Serial.println('w');
	// other targets to reach, lower stack index, reprocess
	if (target > 0) { 
		--target; 
		++process_cycles; 
		layers[LAYER_NAV].active = true;
	}
	// reached last target
	else {
		layers[LAYER_NAV].active = false;
	}
	// get next target by checking current position against game state

	// finished turning in place (2nd call of waypoint)
	if (layers[LAYER_TURN].active) {
		layers[LAYER_NAV].active = true;
		layers[LAYER_TURN].active = false;
	} 
	
}

}