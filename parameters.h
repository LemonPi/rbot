#pragma once
#include <Arduino.h>
#define DEBUGGING

#ifdef DEBUGGING
	#define SERIAL_PRINT(...) Serial.print(__VA_ARGS__);
	#define SERIAL_PRINTLN(...) Serial.println(__VA_ARGS__);
#else
	#define SERIAL_PRINT(...) ;
	#define SERIAL_PRINTLN(...) ;
#endif

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
constexpr byte ALL_ALLOWED = B11111111;

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
constexpr byte RIGHT = 	B0001;
constexpr byte CENTER = B0010;		// center sensor is sensor 0
constexpr byte LEFT = 	B0100;
constexpr byte RED = 	B1000;
constexpr float SIDE_SENSOR_DISTANCE = 52;


// PID speed control
constexpr char FORWARD = 1;
constexpr char BACKWARD = -1;
constexpr char START_DRIFT = 1;	// how many cycles to account for drifting by using previous direction
constexpr char NO_DRIFT = 0;
constexpr int DRIFT_SPEED = 15;	// how fast of a speed change to start accounting for drift

constexpr float BASE_WIDTH = 99.0;
constexpr float BETWEEN_HOPPER_AND_CENTER = 80;
constexpr float TURNING_RADIUS = BASE_WIDTH;
constexpr float RECIPROCAL_BASE_WIDTH = 0.01004009;	// using reciprocal due to faster multiply than divide
constexpr float MM_PER_TICK_L = 0.1714829559*1000/1045;
constexpr float MM_PER_TICK_R = 0.16966084148*1000/1045;
constexpr float L_R_SPEED_RATIO = MM_PER_TICK_R/MM_PER_TICK_R;

constexpr float KP = 1.194;
constexpr float KI = 1.2;
constexpr float KD = 0.005;

constexpr int TPR = 1200;

constexpr int TOP_SPEED = 40; 	// in ticks per cycle 
constexpr int MIN_SPEED = 15;
constexpr int START_SPEED = 45;
constexpr int KICK_SPEED = 55;

constexpr int MOTOR_STALLING = 5; // how many cycles to wait before confirming that motor stalled


// navigation
constexpr float TARGET_CIRCLE = 10.0;	// allow for 20mm error from target
constexpr float TARGET_IMMEDIATE = 5.0;// don't try to get closer than 3mm (fixes high heading error when really close)
constexpr float TARGET_CLOSE = 200.0;	// slow down 100mm from target
constexpr int NAV_TURN = 15;			// turn size in ticks/cycle, adjustable
constexpr float THETA_TOLERANCE = 0.03;	// around 3 degree turning

constexpr int ANY_THETA = 9000;	// if no target is set

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
constexpr byte DEFAULT_LOAD = 7;

constexpr float PILLAR_RADIUS = 24.15;
constexpr float HOPPER_RADIUS = 20.55;
constexpr int COLONY_RADIUS = 150;		// including the 3 pillars
constexpr float OTHER_HOPPER_TOO_CLOSE = COLONY_RADIUS + TURNING_RADIUS + 100;	// how close the target can be to another hopper
constexpr float APPROACH_SCALAR = 2;


// line detecting sensors
constexpr int CALLIBRATION_TIME = 5000;	// 5s
constexpr float THRESHOLD_TOLERANCE = 0.5;
constexpr int LINE_WIDTH = 6;			// about 1cm
constexpr float HALF_LINE_WIDTH = 3;
constexpr int GRID_WIDTH = 200;			// grid spaced about 200mm apart
constexpr int CYCLES_CROSSING_LINE = 2; 	// cycles on line for false positive to fail
constexpr int LINES_PER_CORRECT = 0;	// how many lines to cross before correcting; 0 is every line
constexpr float DISTANCE_CENTER_TO_RED = 4.2;	
constexpr int DISTANCE_CENTER_TO_RED_ALLOWANCE = 35; // allow maximum of 35mm between center hitting and red line leaving

// correct to line directions
constexpr int DIR_UP = 0;
constexpr int DIR_LEFT = -90;
constexpr int DIR_RIGHT = 90;
constexpr int DIR_BACK = 180;


// correction
constexpr byte INTERSECTION_TOO_CLOSE = 40;	// allowed range [50,150] for x and y for a correct
constexpr int CORRECT_TOO_FAR = SIDE_SENSOR_DISTANCE * 0.68;	// correct theta by the distance before all 3 crosses the line
constexpr float CORRECT_CROSSING_TOLERANCE = SIDE_SENSOR_DISTANCE / 6;	// accepted difference in distance travelled between the 2 halves of crossing a line
constexpr float THETA_CORRECT_LIMIT = 0.4;	// don't correct if offset > 23 degrees
constexpr float NEED_TO_HOPPER_CORRECT = 50;

constexpr int PASSED_COOL_DOWN = -10;
constexpr int PASSED_NONE = 0;
constexpr int PASSED_LEFT = LEFT << 3;
constexpr int PASSED_RIGHT = RIGHT << 3;
constexpr int ENCOUNTERED_ALL = B111;

constexpr byte SIDE_LEFT = 1;
constexpr byte SIDE_RIGHT = 2;


// getting the ball
constexpr int GAME_BOARD_X = 1790;
constexpr int GAME_BOARD_Y = 1600;

constexpr int RENDEZVOUS_X = 1350;
constexpr int RENDEZVOUS_Y = 800;
constexpr int RENDEZVOUS_CLOSE = 7;	// within 7cm of rendezvous
constexpr float GET_DISTANCE = 50;
constexpr int GET_SPEED = 0.7*TOP_SPEED;
constexpr int GET_TURN = 10;

// ball statuses
constexpr byte BALL_LESS = 0;
constexpr byte JUST_GOT_BALL = 1;
constexpr byte CAUGHT_BALL = 5;
constexpr byte RELEASED_BALL = 10;
constexpr byte SECURED_BALL = 20;	// cycles of gate closing



// put ball
constexpr byte RELIABLE_CORRECT_CYCLE = 6;
constexpr int PUT_TURN = 10;
constexpr int PUT_TURN_MIN = 5;
}