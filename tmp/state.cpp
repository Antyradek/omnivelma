#include "state.hpp" 

Vels VelsState::getVels()
{
	Vels out;
	out.w1 = get(1);
	out.w2 = get(2);
	out.w3 = get(3);
	out.w4 = get(4);
	return out;
}
