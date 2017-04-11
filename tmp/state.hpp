#pragma once
#include "lalkarz.hpp"

class VelsState
{
public:
	virtual void set(sf::Keyboard::Key key, bool pressed) = 0;
	
	virtual double get(int wheel) = 0;
	
	virtual void reset() = 0;
	
	virtual void update() = 0;
	
	Vels getVels();
};
