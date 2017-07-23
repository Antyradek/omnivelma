#include "gamepad_vels_state.hpp"

void GamepadVelsState::set(sf::Joystick::Axis axis, double position)
{
	switch(axis)
	{
	case AXIS_WHEEL_1:
	case AXIS_WHEEL_1_ALT:
		w1 = -position / 100.0;
		break;
	case AXIS_WHEEL_2:
		w2 = -position / 100.0;
		break;
	case AXIS_WHEEL_3:
		w3 = position / 100.0;
		break;
	case AXIS_WHEEL_4:
	case AXIS_WHEEL_4_ALT:
		w4 = -position / 100.0;
		break;
	default:
		break;
	}
}
