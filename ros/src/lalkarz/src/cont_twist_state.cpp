#include "cont_twist_state.hpp"

ContTwistState::ContTwistState() : DoubleTwistState()
{
	reset();
}


void ContTwistState::reset()
{
	xDir = yDir = zDir = 0;
	x = y = z = 0;
	xClock.restart();
	yClock.restart();
	zClock.restart();
}

void ContTwistState::update()
{
	x = clamp(x + xClock.restart().asSeconds() * xDir * METER_CHANGE_SPEED);
	y = clamp(y + yClock.restart().asSeconds() * yDir * METER_CHANGE_SPEED);
	z = clamp(z + zClock.restart().asSeconds() * zDir * METER_CHANGE_SPEED);
}

double ContTwistState::clamp(double value) const
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

void ContTwistState::set(sf::Keyboard::Key key, bool pressed)
{
	update();
	if(pressed)
	{
		switch(key)
		{
		case KEY_AXIS_X_UP:
			xDir = 1;
			break;
		case KEY_AXIS_X_DOWN:
			xDir = -1;
			break;
		case KEY_AXIS_Y_UP:
			yDir = 1;
			break;
		case KEY_AXIS_Y_DOWN:
			yDir = -1;
			break;
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
		case KEY_AXIS_X_UP:
		case KEY_AXIS_X_DOWN:
			xDir = 0;
			break;
		case KEY_AXIS_Y_UP:
		case KEY_AXIS_Y_DOWN:
			yDir = 0;
			break;
		case KEY_AXIS_Z_LEFT:
		case KEY_AXIS_Z_RIGHT:
			zDir = 0;
			break;
		default:
			break;
		}
	}
}
