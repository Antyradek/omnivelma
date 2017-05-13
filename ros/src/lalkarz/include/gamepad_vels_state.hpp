#pragma once
#include "state.hpp"

///Prędkości kół z kontrolera
class GamepadVelsState : public DoubleVelsState
{
public:
	void set(sf::Joystick::Axis axis, double position) override;
};