#include "rbot.h"
#include "parameters.h"

namespace robot {

void put_ball() {
	Layer& put = layers[LAYER_PUT];
	if (!put.active) return;

	put.speed = 0;
	put.angle = 0;
	// release the ball if you have ball
	if (ball_status == SECURED_BALL) {
		hard_break();
		open_gate();
		--ball_status;
	} 
	// finished releasing the ball, go to waypoint
	else if (ball_status == BALL_LESS) {
		if (paused) resume_drive();
		// open hoppers up for retrieving
		open_hoppers();
		// carry on to next target
		put.active = false;
		waypoint();
	} 
	// in the middle of releasing the ball (release for SECURED_BALL number of cycles)
	else --ball_status;
}



void close_gate() {
	gate.write(170);
	Serial.println("cg");
}
void open_gate() {
	gate.write(70);
	Serial.println("og");
}




}	// end namespace