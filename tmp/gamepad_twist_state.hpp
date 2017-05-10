#pragma once
#include "state.hpp"

class GamepadTwistState : public DoubleTwistState
{
public:
	void set(sf::Joystick::Axis axis, double position) override;
};
