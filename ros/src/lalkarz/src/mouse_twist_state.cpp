#include "mouse_twist_state.hpp"

void MouseTwistState::set(double deltaZ)
{
	z = clamp(z + deltaZ);
}

void MouseTwistState::set(double x, double y)
{
	this -> x = clamp(x);
	this -> y = clamp(y);
}

double MouseTwistState::clamp(double value) const
{
	if(value < -1)
	{
		return -1;
	}
	else if(value > 1)
	{
		return 1;
	}
	return value;
}