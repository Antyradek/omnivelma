#include "mouse_twist_state.hpp"

MouseTwistState::MouseTwistState() : DoubleTwistState()
{
	reset();
}

void MouseTwistState::reset()
{
	zDir = 0;
	x = y = z = 0;
	zClock.restart();
}

void MouseTwistState::update()
{
	z = clamp(z + zClock.restart().asSeconds() * zDir * METER_CHANGE_SPEED);
}

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

void MouseTwistState::set(sf::Keyboard::Key key, bool pressed)
{
	update();
	if(pressed)
	{
		switch(key)
		{
		case KEY_AXIS_Z_LEFT:
			zDir = 1;
			break;
		case KEY_AXIS_Z_RIGHT:
			zDir = -1;
			break;
		default:
			break;
		}
	}
	else
	{
		switch(key)
		{
		case KEY_AXIS_Z_LEFT:
		case KEY_AXIS_Z_RIGHT:
			zDir = 0;
			break;
		default:
			break;
		}
	}
}
