#pragma once
#include "state.hpp"

///Stopniowe ustawianie prędkości
class StepsVelsState : public IntVelsState
{
protected:
	int clamp(int value) const;
public:
	void set(sf::Keyboard::Key key, bool pressed) override;
	double get(Wheel wheel) override;
};

///Stopniowe ustawianie prędkości z podtrzymaniem na zerze
class StepsVelsStateHold : public StepsVelsState
{
	double get(Wheel wheel) override;
};
