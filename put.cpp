#include "rbot.h"
#include "parameters.h"

namespace robot {

void put_ball() {
	Layer& put = layers[LAYER_PUT];
	if (!put.active) return;

	put.speed = 0;
	// turn to play ball
	// compared to 0 degrees facing gbot
	int turn_speed = (-theta) * 2 * PUT_TURN; 
	if (abs(theta) < 2*THETA_TOLERANCE) {
		++turned_to_put;
		put.angle = 0;
	}
	else {
		put.angle = turn_speed;
		if (turn_speed < 0) put.angle -= 0.3*MIN_SPEED;
		else put.angle += 0.3*MIN_SPEED;
		turned_to_put = 0;
	}
	// SERIAL_PRINTLN(turned_to_put);
	// SERIAL_PRINT('b');
	// SERIAL_PRINTLN(ball_status);
	if (turned_to_put > RELIABLE_CORRECT_CYCLE) {
		put.angle = 0;
		// release the ball if you have ball
		if (ball_status == SECURED_BALL) {
			hard_break(LAYER_PUT);
			open_gate();
			--ball_status;
		} 
		// finished releasing the ball, go to 	
		else if (ball_status == BALL_LESS) {
			if (paused) resume_drive(LAYER_PUT);
			// open hoppers up for retrieving
			open_hoppers();
			// carry on to next target
			put.active = false;
			turned_to_put = 0;
			waypoint(LAYER_PUT);
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