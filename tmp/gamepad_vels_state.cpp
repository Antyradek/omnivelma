#include "gamepad_vels_state.hpp" 

GamepadVelsState::GamepadVelsState()
{
	w1 = w2 = w3 = w4 = 0;
}

double GamepadVelsState::get(int wheel)
{
	switch(wheel)
	{
		case 1:
			return w1;
		case 2:
			return w2;
		case 3:
			return w3;
		case 4:
			return w4;
		default:
			return 0;
	}
}

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

void GamepadVelsState::reset()
{
	w1 = w2 = w3 = w4 = 0;
}

