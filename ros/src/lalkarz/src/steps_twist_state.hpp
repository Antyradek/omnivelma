#pragma once
#include "state.hpp"

///Stopniowe ustawianie kierunku
class StepsTwistState : public IntTwistState
{
protected:
	int clamp(int value) const;
public:
	void set(sf::Keyboard::Key key, bool pressed) override;
	double getAxis(Axis axis) override;

};