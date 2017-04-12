#pragma once
#include "lalkarz.hpp"

class VelsState
{
public:
	virtual void set(sf::Keyboard::Key key, bool pressed);
	
	virtual void set(sf::Joystick::Axis axis, double position);
	
	virtual double get(int wheel);
	
	virtual void reset();
	
	virtual void update();
	
	Vels getVels();
};