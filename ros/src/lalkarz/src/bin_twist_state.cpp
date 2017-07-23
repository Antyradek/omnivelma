#include "bin_twist_state.hpp"

void BinTwistState::set(sf::Keyboard::Key key, bool pressed)
{
	if(pressed)
	{
		switch(key)
		{
		case KEY_AXIS_X_UP:
			x = 1;
			break;
		case KEY_AXIS_X_DOWN:
			x = -1;
			break;
		case KEY_AXIS_Y_UP:
			y = 1;
			break;
		case KEY_AXIS_Y_DOWN:
			y = -1;
			break;
		case KEY_AXIS_Z_LEFT:
			z = 1;
			break;
		case KEY_AXIS_Z_RIGHT:
			z = -1;
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
			x = 0;
			break;
		case KEY_AXIS_Y_UP:
		case KEY_AXIS_Y_DOWN:
			y = 0;
			break;
		case KEY_AXIS_Z_LEFT:
		case KEY_AXIS_Z_RIGHT:
			z = 0;
			break;
		default:
			break;
		}
	}
}

double BinTwistState::getAxis(Axis axis)
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

