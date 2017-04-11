#pragma once
#include "state.hpp"

class StepsVelsState : public VelsState
{
protected:
	int w1;
	int w2;
	int w3;
	int w4;
	
	int clamp(int value) const;
public:
	StepsVelsState();
	void set(sf::Keyboard::Key key, bool pressed) override;
	void update() override;
	void reset() override;
	double get(int wheel) override;
};

class StepsVelsStateHold : public StepsVelsState
{
	double get(int wheel) override;
};
