#include "rbot.h"
#include "parameters.h"

namespace robot {

void put_ball() {
	Layer& put = layers[LAYER_PUT];
	if (!put.active) return;
	stop();
}

void close_gate() {
	gate.write(170);
}
void open_gate() {
	gate.write(70);
}




}	// end namespace