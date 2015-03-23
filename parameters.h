#pragma once
#include <Arduino.h>

namespace robot {
// mathematical constants
constexpr float TWOPI = 6.2831853070;
constexpr float HALFPI = 1.5707963268;
constexpr float QUARTERPI = 0.78539816339;
constexpr float DEGS = 0.0174532925;
constexpr float RADS = 57.29577951;

// drive modes
constexpr bool MANUAL = false;
constexpr bool AUTOMATIC = true;

// subsumption layers
constexpr byte LAYER_NUM = 6;
constexpr byte LAYER_BOUND = 0; // highest priority
constexpr byte LAYER_TURN = 1;
constexpr byte LAYER_NAV = 2;
constexpr byte LAYER_GET = 3;	// retrieve from hopper
constexpr byte LAYER_PUT = 4;	// deposit ball to rendevous
constexpr byte LAYER_WAIT = 5;

constexpr int CYCLE_TIME = 50; 	// in ms
constexpr int SENSOR_TIME = 10;  // in ms, 5x faster than navigation cycles

// maximum array bounds
constexpr byte BOUNDARY_MAX = 14;
constexpr byte TARGET_MAX = 10;
constexpr byte SENSOR_MAX = 4;

// target types
constexpr byte TARGET_NAV = 0;
constexpr byte TARGET_COR = 1;
constexpr byte TARGET_TURN = 2;
constexpr byte TARGET_GET = 3;
constexpr byte TARGET_PUT = 4;

// sensor indices
constexpr byte CENTER = B0001;		// center sensor is sensor 0
constexpr byte LEFT = 	B0010;
constexpr byte RIGHT = 	B0100;
constexpr byte BALL = 	B1000;
constexpr float SIDE_SENSOR_DISTANCE = 43.5;


// PID speed control
constexpr int FORWARD = 1;
constexpr int BACKWARD = -1;

constexpr float BASE_WIDTH = 99.0;
constexpr float TURNING_RADIUS = BASE_WIDTH;
constexpr float RECIPROCAL_BASE_WIDTH = 0.01004009;	// using reciprocal due to faster multiply than divide
constexpr float MM_PER_TICK_L = 0.1714829559*1000/1045;
constexpr float MM_PER_TICK_R = 0.16966084148*1000/1045;

constexpr float KP = 1.194;
constexpr float KI = 1.2;
constexpr float KD = 0.005;

constexpr int TPR = 1200;

constexpr int TOP_SPEED = 45; 	// in ticks per cycle 
constexpr int MIN_SPEED = 20;
constexpr int START_SPEED = 45;


// navigation
constexpr float TARGET_CIRCLE = 10.0;	// allow for 20mm error from target
constexpr float TARGET_IMMEDIATE = 5.0;// don't try to get closer than 3mm (fixes high heading error when really close)
constexpr float TARGET_CLOSE = 200.0;	// slow down 100mm from target
constexpr int NAV_TURN = 20;			// turn size in ticks/cycle, adjustable
constexpr float THETA_TOLERANCE = 0.03;	// around 3 degree turning

constexpr int ANY_THETA = 9000;	// if no target is set

constexpr int TURNING_IN_PLACE = 8000;
constexpr float CAN_TURN_IN_PLACE = 0.5; // minimum angle to activate turning in place



// boundary avoidance
constexpr int BOUNDARY_TOO_CLOSE = 200;	// 200 mm from center of wheels
constexpr int BOUNDARY_FAR_ENOUGH = 100;	// can stop tracking active boundary this far away
constexpr float BOUNDARY_TOLERANCE = HALFPI; // corresponds to how wide it is
constexpr float EXISTENTIAL_THREAT = 0.3*BOUNDARY_TOO_CLOSE;

constexpr int BOUND_TURN = 20;		// how hard to turn away from obstacle; adjustable

constexpr int NONE_ACTIVE = -1;

// hopper indices, 3-hoppers are loaded first
constexpr byte HOPPER_NUM = 4;
constexpr byte HOPPER1 = 3;
constexpr byte HOPPER2 = 7;
constexpr byte HOPPER3 = 10;
constexpr byte HOPPER4 = 13;

constexpr float PILLAR_RADIUS = 24.15;
constexpr float HOPPER_RADIUS = 20.55;
constexpr int COLONY_RADIUS = 150;		// including the 3 pillars
constexpr float OTHER_HOPPER_TOO_CLOSE = COLONY_RADIUS + TURNING_RADIUS + 100;	// how close the target can be to another hopper


// line detecting sensors
constexpr int CALLIBRATION_TIME = 5000;	// 5s
constexpr int THRESHOLD_TOLERANCE = 3;
constexpr int LINE_WIDTH = 9; 			// about 1cm
constexpr float HALF_LINE_WIDTH = 4.5;
constexpr int GRID_WIDTH = 200;			// grid spaced about 200mm apart
constexpr int CYCLES_CROSSING_LINE = 2; 	// cycles on line for false positive to fail
constexpr int LINES_PER_CORRECT = 0;	// how many lines to cross before correcting; 0 is every line

// correct to line directions
constexpr int DIR_UP = 0;
constexpr int DIR_LEFT = -90;
constexpr int DIR_RIGHT = 90;
constexpr int DIR_BACK = 180;


// correction
constexpr int CORRECT_SPEED = 10;			// one wheel travels at 0 and the other 2*CORRECT_SPEED
constexpr byte INTERSECTION_TOO_CLOSE = 40;	// allowed range [50,150] for x and y for a correct
constexpr int CORRECT_TOO_FAR = 40;	// correct theta by the distance before all 3 crosses the line
constexpr float CORRECT_CROSSING_TOLERANCE = 4;	// accepted difference in distance travelled between the 2 halves of crossing a line

constexpr int PASSED_NONE = 0;
constexpr int PASSED_LEFT = LEFT << 3;
constexpr int PASSED_RIGHT = RIGHT << 3;
constexpr int PASSED_LEFT_RIGHT = B010101;	// left passed, right active, center active
constexpr int PASSED_RIGHT_LEFT = B100011;	// right passed, left active, center active



// getting the ball
constexpr int RENDEZVOUS_X = 1400;
constexpr int RENDEZVOUS_Y = 800;
constexpr int RENDEZVOUS_CLOSE = 40;	// within 4cm of rendezvous
constexpr float GET_DISTANCE = 50;
constexpr int GET_SPEED = 0.5*TOP_SPEED;
// ball statuses
constexpr byte BALL_LESS = 0;
constexpr byte JUST_GOT_BALL = 1;
constexpr byte SECURED_BALL = 15;	// cycles of gate closing

}