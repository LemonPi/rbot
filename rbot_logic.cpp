#include "rbot.h"
#include "parameters.h"

namespace robot {

bool has_ball = false;

// called inside every go cycle
void user_behaviours() {
	correct_theta();
}

// called after arriving
void user_waypoint() {
	
}


}	// end namespace