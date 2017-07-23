#include "steps_twist_state.hpp"

int StepsTwistState::clamp(int value) const
{
	if(value < -INPUT_STEP_COUNT)
	{
		return -INPUT_STEP_COUNT;
	}
	if(value > INPUT_STEP_COUNT)
	{
		return INPUT_STEP_COUNT;
	}
	return value;
}

double StepsTwistState::getAxis(Axis axis)
{
	switch(axis)
	{
	case X:
		return (double)x / INPUT_STEP_COUNT;
	case Y:
		return (double)y / INPUT_STEP_COUNT;
	case Z:
		return (double)z / INPUT_STEP_COUNT;
	default:
		return 0;
	}
}

void StepsTwistState::set(sf::Keyboard::Key key, bool pressed)
{
	if(pressed)
	{
		switch(key)
		{
		case KEY_AXIS_X_UP:
			x += 1;
			break;
		case KEY_AXIS_X_DOWN:
			x -= 1;
			break;
		case KEY_AXIS_Y_UP:
			y += 1;
			break;
		case KEY_AXIS_Y_DOWN:
			y -= 1;
			break;
		case KEY_AXIS_Z_LEFT:
			z += 1;
			break;
		case KEY_AXIS_Z_RIGHT:
			z -= 1;
			break;
		default:
			break;
		}
		x = clamp(x);
		y = clamp(y);
		z = clamp(z);
	}
}


