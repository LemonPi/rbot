#pragma once
#include <Arduino.h>
#include <hbridge.h>
#include "parameters.h"

namespace robot {

// each subsumption layer has a command, argument, and active
struct Layer {
	// speed and angle in units of ticks/cycle
	int speed, angle, active;
};
// each location has x and y coordinates
struct Boundary {
	double x, y, r, distance, theta, threat;
};
// each target has x and y coordinates in addition to optional theta
struct Target {
	double x, y, theta;
};

// 2 wheel differentially driven robot
// left input is connected to pin 2, right input is connected to pin 3
extern volatile int tick_l, tick_r;		// time between ticks
extern int instant_tick_l, instant_tick_r;
// subsumption layers
extern Layer layers[LAYER_NUM];

// avoid boundaries (point boundaries)
extern Boundary boundaries[BOUNDARY_MAX];
extern int boundary_num, active_boundary;

// PID information
extern unsigned long time_prev;
extern double integral_l, integral_r;
extern int prev_l, prev_r;
extern int target_l, target_r;	// target time between ticks
extern int out_l, out_r;			// output values

// internal coordinates
extern double x, y, theta, tot_distance;	

// waypoint navigation
extern Target targets[TARGET_MAX];
extern double target_distance, last_target_distance, heading_error;
extern int target;	// target index
extern int process_cycles;

// motor control output hbridges
extern Hbridge l, r;		
// 1 for forward, -1 for backward
extern int dir_l, dir_r;

// line detection for navigation correction
extern unsigned long time_prev_sensors;
extern int sensor_num;
extern byte sensors[SENSOR_MAX];
extern byte indicators[SENSOR_MAX];
extern int readings[SENSOR_MAX];
extern int thresholds[SENSOR_MAX];
extern int cycles_on_line, counted_lines;
extern bool square_turn;
extern bool deviate_from_line;
extern float pre_deviate_distance;
extern byte side_correct;

extern bool drive, on;

// methods
void tick_left();
void tick_right();

bool on_line(byte pin);
bool is_intersection(int x, int y);
// restrict values between low and high limits
void clamp(int& parameter, int low, int high);
void pid_control(int tl, int tr);
// update internal position and call speed control
void odometry();
// correct proprioception: position with line detection
void line_detect();
// target acquisition
void locate_target();
// navigating to target
void navigate();
// what to do when arriving at 
void waypoint();
// turning at waypoints
void hard_turn();
// avoiding boundaries
void avoid_boundary();


// decide which is the highest active layer to pass to motor control
void arbitrate();
void motor_control(byte layer);


void user_behaviours();
void user_waypoint();



// 3 pins each for hbridges, by default off and going forward
void initialize_robot(byte c1_l, byte c2_l, byte outpin_l, byte c1_r, byte c2_r, byte outpin_r);

void start();
void stop();

int get_active_layer();

void set_coordinate(double tx, double ty, double td = 0.0);
void set_drive(bool mode);

int add_target(double tx, double ty, double td = ANY_THETA, bool rad = false);
int add_boundary(double bx, double by, double radius = 0);	// by default a point
int add_sensor(byte sensor_pin, byte indicator_pin = -1);	// -1 means no indicator associated with sensor


bool go();	// base user to be called in loop
bool correct(); // correct lines to be called in a loop

// calibrate the sensors
void calibrate();
void indicate_sensors();
void correct_to_grid();
void correct_to_line();



// retrieve information from the robot
bool get_on();
double get_x();
double get_y();
double get_theta();

int get_active_target();
const Target& get_target();

int get_direction_l();
int get_direction_r();

double get_target_distance();
double get_heading_error();


int get_boundary_num();
const Boundary& get_boundary(int b);




}	// end namespace
