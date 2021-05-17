#ifndef ROBOCALC_DATATYPES_H_
#define ROBOCALC_DATATYPES_H_

#include <string>
#include <set>
#include <tuple>
 
enum State {
	Ramp, Init, Wait24Vpower, ClosedLoop, ErrorMode
};
enum Power {
	On, Off
};

#endif
