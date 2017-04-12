#pragma once
#include "state.hpp"

class GamepadVelsState : public VelsState
{
protected:
	double w1;
	double w2;
	double w3;
	double w4;
public:
	GamepadVelsState();
	void set(sf::Joystick::Axis axis, double position) override;
	double get(int wheel) override;
	void reset() override;
};