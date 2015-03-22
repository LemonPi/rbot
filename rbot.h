#include <Adafruit_TiCoServo.h>
#include "parameters.h"
#include "robot.h"

namespace robot {

extern byte ball_status;
extern bool getting_ball;
extern Adafruit_TiCoServo gate;
extern float get_initial_distance;
extern byte hoppers[HOPPER_NUM];

void initialize_rbot(byte servo_pin);

// get module
void get_ball();
void close_gate();

void add_hopper(byte p1, byte p2, byte p3);
Target approach_hopper(byte hopper);

void open_hoppers();
void close_hoppers();

// put module (deposit ball to gbot)
void open_gate();

// user correct
// correct theta at lines
void correct_theta();


}