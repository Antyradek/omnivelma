#include "state.hpp" 

Vels State::getVels()
{
	Vels out;
	out.w1 = get(Wheel::W1);
	out.w2 = get(Wheel::W2);
	out.w3 = get(Wheel::W3);
	out.w4 = get(Wheel::W4);
	return out;
}

Twist State::getTwist()
{
	Twist out;
	out.x = getAxis(Axis::X);
	out.y = getAxis(Axis::Y);
	out.z = getAxis(Axis::Z);
	return out;
}

double State::get(Wheel wheel)
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

double State::getAxis(Axis axis)
{
	return 0;
}


void State::set(double deltaZ)
{
}

void State::set(double x, double y)
{
}

IntVelsState::IntVelsState()
{
	reset();
}

void IntVelsState::reset()
{
	w1 = w2 = w3 = w4 = 0;
}

IntTwistState::IntTwistState()
{
	reset();
}

void IntTwistState::reset()
{
	x = y = z = 0;
}

DoubleVelsState::DoubleVelsState()
{
	reset();
}

void DoubleVelsState::reset()
{
	w1 = w2 = w3 = w4 = 0;
}

double DoubleVelsState::get(Wheel wheel)
{
	switch(wheel)
	{
		case W1:
			return w1;
		case W2:
			return w2;
		case W3:
			return w3;
		case W4:
			return w4;
		default:
			return 0;
	}
}
