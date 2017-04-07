#pragma once
#include "lalkarz.hpp"

class VelsState
{
public:
	void set(sf::Keyboard::Key key, bool pressed);
	
	double get(int wheel);
	
	void reset();
	
private:
	std::mutex mainMutex;
	
	virtual void setDef(sf::Keyboard::Key key, bool pressed) = 0;
	
	virtual double getDef(int wheel) = 0;
	
	virtual void resetDef() = 0;
};
