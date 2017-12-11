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

std::vector<double> State::getGears()
{
	return gears;
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
	std::vector<double> newGears {0.01, 0.05, 0.1, 0.5, 1, 2, 5};
	gears = newGears;
	reset();
}

void IntVelsState::reset()
{
	w1 = w2 = w3 = w4 = 0;
}

IntTwistState::IntTwistState()
{
	std::vector<double> newGears {0.001, 0.005, 0.01, 0.05, 0.1, 0.2, 0.5};
	gears = newGears;
	reset();
}

void IntTwistState::reset()
{
	x = y = z = 0;
}

DoubleVelsState::DoubleVelsState()
{
	std::vector<double> newGears {0.01, 0.05, 0.1, 0.5, 1, 2, 5};
	gears = newGears;
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

void DoubleTwistState::reset()
{
	std::vector<double> newGears {0.001, 0.005, 0.01, 0.05, 0.1, 0.2, 0.5};
	gears = newGears;
	x = y = z = 0;
}

DoubleTwistState::DoubleTwistState()
{
	reset();
}

double DoubleTwistState::getAxis(Axis axis)
{
	switch(axis)
	{
	case X:
		return x;
	case Y:
		return y;
	case Z:
		return z;
	default:
		return 0;
	}
}

