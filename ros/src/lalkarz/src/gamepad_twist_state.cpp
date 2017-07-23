#include "gamepad_twist_state.hpp"

void GamepadTwistState::set(sf::Joystick::Axis axis, double position)
{
	switch(axis)
	{
	case AXIS_TWIST_X:
	case AXIS_TWIST_X_ALT:
		x = position / 100.0;
		break;
	case AXIS_TWIST_Y:
	case AXIS_TWIST_Y_ALT:
		y = -position / 100.0;
		break;
	case AXIS_TWIST_Z:
		z = -position / 100.0;
		break;
	default:
		break;
	}
}

