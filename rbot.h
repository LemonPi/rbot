#include "parameters.h"
#include "robot.h"

namespace robot {

extern bool has_ball;

// get module
void add_hopper(byte p1, byte p2, byte p3);
Target approach_hopper(byte hopper);

void open_hoppers();
void close_hoppers();

// user correct
// correct theta at lines
void correct_theta();

}