#pragma once
// mathematical constants
#define TWOPI 6.2831853070
#define PI 3.1415926536
#define HALFPI 1.5707963268

// subsumption layers
#define LAYER_NUM 5
#define LAYER_BUMP 0	// highest priority
#define LAYER_BOUND 1
#define LAYER_TURN 2
#define LAYER_NAV 3
#define LAYER_WAIT 4

#define CYCLE_TIME 50 	// in ms
#define SENSOR_TIME 10  // in ms, 5x faster than navigation cycles

// maximum array bounds
#define BOUNDARY_MAX 14
#define TARGET_MAX 10
#define SENSOR_MAX 4



// sensor indices
#define CENTER 0		// center sensor is sensor 0
#define LEFT 1
#define RIGHT 2
#define BALL 3

// PID speed control
#define FORWARD 1
#define BACKWARD (-1)

#define BASE_WIDTH 99.0
#define RECIPROCAL_BASE_WIDTH 0.01004009	// using reciprocal due to faster multiply than divide
#define MM_PER_TICK_L 0.1721899559
#define MM_PER_TICK_R 0.16866084148

#define KP 1.194
#define KI 1.2
#define KD 0.005

#define TPR 1200

#define TOP_SPEED 55 	// in ticks per cycle 
#define MIN_SPEED 20
#define START_SPEED 55


// navigation
#define TARGET_CIRCLE 10.0	// allow for 20mm error from target
#define TARGET_IMMEDIATE 5.0// don't try to get closer than 3mm (fixes high heading error when really close)
#define TARGET_CLOSE 200.0	// slow down 100mm from target
#define NAV_TURN 20			// turn size in ticks/cycle, adjustable
#define THETA_TOLERANCE 0.03	// around 3 degree turning

#define ANY_THETA 9000	// if no target is set

#define TURNING_IN_PLACE 8000
#define CAN_TURN_IN_PLACE 0.5 // minimum angle to activate turning in place



// boundary avoidance
#define BOUNDARY_TOO_CLOSE 200	// 200 mm from center of wheels
#define BOUNDARY_FAR_ENOUGH 100	// can stop tracking active boundary this far away
#define BOUNDARY_TOLERANCE HALFPI // corresponds to how wide it is
#define EXISTENTIAL_THREAT 0.3*BOUNDARY_TOO_CLOSE

#define BOUND_TURN 20		// how hard to turn away from obstacle; adjustable

#define NONE_ACTIVE -1

// hopper indices, 3-hoppers are loaded first
#define HOPPER1 3
#define HOPPER2 7
#define HOPPER3 10
#define HOPPER4 13

#define PILLAR_RADIUS 24.15
#define HOPPER_RADIUS 20.55
#define COLONY_RADIUS 150

// line detecting sensors
#define CALLIBRATION_TIME 7000	// 5s
#define LINE_WIDTH 10 			// about 1cm
#define GRID_WIDTH 200			// grid spaced about 200mm apart
#define CYCLES_ON_LINE 2 		// 
#define LINES_PER_CORRECT 3