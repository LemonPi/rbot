#include "rbot.h"
#include "parameters.h"

namespace robot {

void close_gate() {
	gate.write(170);
}
void open_gate() {
	gate.write(70);
}




}	// end namespace