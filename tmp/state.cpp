#include "state.hpp" 

Vels State::getVels()
{
	Vels out;
	out.w1 = get(1);
	out.w2 = get(2);
	out.w3 = get(3);
	out.w4 = get(4);
	return out;
}

Twist State::getTwist()
{
	Twist out;
	out.x = getAxis(0);
	out.y = getAxis(1);
	out.z = getAxis(2);
	return out;
}

double State::get(int wheel)
{ 
	return 0;
}

void State::reset()
{
}

void State::set(sf::Joystick::Axis axis, double position)
{
}

void State::set(sf::Keyboard::Key key, bool pressed)
{
}

void State::update()
{
}

double State::getAxis(int axis)
{
	return 0;
}


void State::set(double deltaZ)
{
}

void State::set(double x, double y)
{
}

