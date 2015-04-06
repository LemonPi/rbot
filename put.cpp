#include "rbot.h"
#include "parameters.h"

namespace robot {

void put_ball() {
	Layer& put = layers[LAYER_PUT];
	if (!put.active) return;

	// turn to play ball
	// compared to 0 degrees facing gbot
	int turn_speed = (-theta) * 3 * PUT_TURN; 
	if (abs(theta) < THETA_TOLERANCE || abs(turn_speed) < PUT_TURN_MIN) {
		++turned_to_put;
	}
	else {
		put.angle = turn_speed;
		turned_to_put = 0;
	}

	if (turned_to_put > RELIABLE_CORRECT_CYCLE) {
		put.speed = 0;
		put.angle = 0;
		// release the ball if you have ball
		if (ball_status == SECURED_BALL) {
			hard_break(LAYER_PUT);
			open_gate();
			--ball_status;
		} 
		// finished releasing the ball, go to waypoint
		else if (ball_status == BALL_LESS) {
			if (paused) resume_drive(LAYER_PUT);
			// open hoppers up for retrieving
			open_hoppers();
			// carry on to next target
			put.active = false;
			turned_to_put = 0;
			waypoint();
		} 
		// in the middle of releasing the ball (release for SECURED_BALL number of cycles)
		else --ball_status;
	}
}



void close_gate() {
	gate.write(160);
	SERIAL_PRINTLN("cg");
}
void open_gate() {
	gate.write(60);
	SERIAL_PRINTLN("og");
}




}	// end namespace