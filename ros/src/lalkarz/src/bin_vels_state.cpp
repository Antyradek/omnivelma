#include "bin_vels_state.hpp"

void BinVelsState::set(sf::Keyboard::Key key, bool pressed)
{
	if(pressed)
	{
		switch(key)
		{
		case KEY_WHEEL_1_UP:
			w1 = 1;
			break;
		case KEY_WHEEL_1_DOWN:
			w1 = -1;
			break;
		case KEY_WHEEL_2_UP:
			w2 = 1;
			break;
		case KEY_WHEEL_2_DOWN:
			w2 = -1;
			break;
		case KEY_WHEEL_3_UP:
			w3 = 1;
			break;
		case KEY_WHEEL_3_DOWN:
			w3 = -1;
			break;
		case KEY_WHEEL_4_UP:
			w4 = 1;
			break;
		case KEY_WHEEL_4_DOWN:
			w4 = -1;
			break;
		default:
			break;
		}
	}
	else
	{
		switch(key)
		{
		case KEY_WHEEL_1_UP:
		case KEY_WHEEL_1_DOWN:
			w1 = 0;
			break;
		case KEY_WHEEL_2_UP:
		case KEY_WHEEL_2_DOWN:
			w2 = 0;
			break;
		case KEY_WHEEL_3_UP:
		case KEY_WHEEL_3_DOWN:
			w3 = 0;
			break;
		case KEY_WHEEL_4_UP:
		case KEY_WHEEL_4_DOWN:
			w4 = 0;
			break;
		default:
			break;
		}
	}
}

double BinVelsState::get(Wheel wheel)
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


double BinVelsStateHold::get(Wheel wheel)
{
	switch(wheel)
	{
	case W1:
		return (w1 == 0) ? std::nan("") : w1;
	case W2:
		return (w2 == 0) ? std::nan("") : w2;
	case W3:
		return (w3 == 0) ? std::nan("") : w3;
	case W4:
		return (w4 == 0) ? std::nan("") : w4;
	default:
		return 0;
	}
}

