#pragma once
#include "state.hpp"

///Binarne sterowanie kierunkiem
class BinTwistState : public IntTwistState
{
public:
	void set(sf::Keyboard::Key key, bool pressed) override;
	double getAxis(Axis axis) override;
};