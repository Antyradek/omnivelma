#include "cont_vels_state.hpp"

ContVelsState::ContVelsState() : DoubleVelsState()
{
	reset();
}

void ContVelsState::reset()
{
	w1dir = w2dir = w3dir = w4dir = 0;
	w1 = w2 = w3 = w4 = 0;
}

void ContVelsState::set(sf::Keyboard::Key key, bool pressed)
{
	if(pressed)
	{
		switch(key)
		{
			case KEY_WHEEL_1_UP:
				w1 += w1clock.restart().asSeconds() * w1dir * METER_CHANGE_SPEED;
				w1dir = 1;
				break;
			case KEY_WHEEL_1_DOWN:
				w1 += w1clock.restart().asSeconds() * w1dir * METER_CHANGE_SPEED;
				w1dir = -1;
				break;
			case KEY_WHEEL_2_UP:
				w2 += w2clock.restart().asSeconds() * w2dir * METER_CHANGE_SPEED;
				w2dir = 1;
				break;
			case KEY_WHEEL_2_DOWN:
				w2 += w2clock.restart().asSeconds() * w2dir * METER_CHANGE_SPEED;
				w2dir = -1;
				break;
			case KEY_WHEEL_3_UP:
				w3 += w3clock.restart().asSeconds() * w3dir * METER_CHANGE_SPEED;
				w3dir = 1;
				break;
			case KEY_WHEEL_3_DOWN:
				w3 += w3clock.restart().asSeconds() * w3dir * METER_CHANGE_SPEED;
				w3dir = -1;
				break;
			case KEY_WHEEL_4_UP:
				w4 += w4clock.restart().asSeconds() * w4dir * METER_CHANGE_SPEED;
				w4dir = 1;
				break;
			case KEY_WHEEL_4_DOWN:
				w4 += w4clock.restart().asSeconds() * w4dir * METER_CHANGE_SPEED;
				w4dir = -1;
				break;
			default:
				break;
		}
	}
	else
	{
		switch(key)
		{
			case KEY_WHEEL_1_DOWN:
			case KEY_WHEEL_1_UP:
				w1 += w1clock.restart().asSeconds() * w1dir * METER_CHANGE_SPEED;
				w1dir = 0;
				break;
			case KEY_WHEEL_2_UP:
			case KEY_WHEEL_2_DOWN:
				w2 += w2clock.restart().asSeconds() * w2dir * METER_CHANGE_SPEED;
				w2dir = 0;
				break;
			case KEY_WHEEL_3_UP:
			case KEY_WHEEL_3_DOWN:
				w3 += w3clock.restart().asSeconds() * w3dir * METER_CHANGE_SPEED;
				w3dir = 0;
				break;
			case KEY_WHEEL_4_UP:
			case KEY_WHEEL_4_DOWN:
				w4 += w4clock.restart().asSeconds() * w4dir * METER_CHANGE_SPEED;
				w4dir = 0;
				break;
			default:
				break;
				
		}
	}
	
	w1 = clamp(w1);
	w2 = clamp(w2);
	w3 = clamp(w3);
	w4 = clamp(w4);
}

void ContVelsState::update()
{
	w1 = clamp(w1 + w1clock.restart().asSeconds() * w1dir * METER_CHANGE_SPEED);
	w2 = clamp(w2 + w2clock.restart().asSeconds() * w2dir * METER_CHANGE_SPEED);
	w3 = clamp(w3 + w3clock.restart().asSeconds() * w3dir * METER_CHANGE_SPEED);
	w4 = clamp(w4 + w4clock.restart().asSeconds() * w4dir * METER_CHANGE_SPEED);
	
}

double ContVelsState::clamp(double value) const
{
	if(value < -1)
	{
		return -1;
	}
	if(value > 1)
	{
		return 1;
	}
	return value;
}



