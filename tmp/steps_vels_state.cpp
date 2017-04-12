#include "steps_vels_state.hpp"

StepsVelsState::StepsVelsState()
{
	w1 = w2 = w3 = w4 = 0;
}

void StepsVelsState::set(sf::Keyboard::Key key, bool pressed)
{
	if(pressed)
	{
		
		switch(key)
		{
			case KEY_WHEEL_1_UP:
				w1 += 1;
				break;
			case KEY_WHEEL_1_DOWN:
				w1 -= 1;
				break;
			case KEY_WHEEL_2_UP:
				w2 += 1;
				break;
			case KEY_WHEEL_2_DOWN:
				w2 -= 1;
				break;
			case KEY_WHEEL_3_UP:
				w3 += 1;
				break;
			case KEY_WHEEL_3_DOWN:
				w3 -= 1;
				break;
			case KEY_WHEEL_4_UP:
				w4 += 1;
				break;
			case KEY_WHEEL_4_DOWN:
				w4 -= 1;
				break;
			default:
				break;
		}
		w1 = clamp(w1);
		w2 = clamp(w2);
		w3 = clamp(w3);
		w4 = clamp(w4);
		
	}
}

void StepsVelsState::reset()
{
	w1 = w2 = w3 = w4 = 0;
}

void StepsVelsState::update()
{
	
}

double StepsVelsState::get(int wheel)
{
	switch(wheel)
	{
		case 1:
			return (double)w1 / INPUT_STEP_COUNT;
		case 2:
			return (double)w2 / INPUT_STEP_COUNT;
		case 3:
			return (double)w3 / INPUT_STEP_COUNT;
		case 4:
			return (double)w4 / INPUT_STEP_COUNT;
		default:
			return 0;
	}
}

int StepsVelsState::clamp(int value) const
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

double StepsVelsStateHold::get(int wheel)
{
	switch(wheel)
	{
		case 1:
			return (w1 == 0) ? std::nan("") : (double)w1 / INPUT_STEP_COUNT;
		case 2:
			return (w2 == 0) ? std::nan("") : (double)w2 / INPUT_STEP_COUNT;
		case 3:
			return (w3 == 0) ? std::nan("") : (double)w3 / INPUT_STEP_COUNT;
		case 4:
			return (w4 == 0) ? std::nan("") : (double)w4 / INPUT_STEP_COUNT;
		default:
			return 0;
	}
}
