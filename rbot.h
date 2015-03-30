#include <Adafruit_TiCoServo.h>
#include "parameters.h"
#include "robot.h"

namespace robot {

struct Hopper {
	byte index, load;
};

extern byte ball_pin;

extern byte ball_status;
extern bool getting_ball;
extern Adafruit_TiCoServo gate;
extern float get_initial_distance;
extern Hopper hoppers[HOPPER_NUM];
extern byte active_hopper;
extern byte hit_first;

extern int cycles_on_line, counted_lines;
extern bool corrected_while_backing_up;

extern int passive_status;
extern float correct_initial_distance;
extern float correct_half_distance;


extern int cycles_on_line, counted_lines;
// number of lines crossed
extern int left_crossed, right_crossed;
extern byte center_status;

extern float last_correct_distance;


void initialize_rbot(byte servo_pin, byte ball_proxity_pin);

// get module
void get_ball();
void close_gate();

bool caught_ball();

void add_hopper(byte p1, byte p2, byte p3, byte load = DEFAULT_LOAD);
Target approach_hopper(byte hopper);
byte hopper_select(byte hopper, byte exclude = 200);

void open_hoppers();
void close_hoppers();

// put module (deposit ball to gbot)
void put_ball();
void open_gate();


// user correct
// correct theta at lines
void passive_correct();
void passive_position_correct();
void correct_to_hopper();

}