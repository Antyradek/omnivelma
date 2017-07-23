#pragma once
#include "state.hpp"

///Binarne wejście
class BinVelsState : public IntVelsState
{
public:
	void set(sf::Keyboard::Key key, bool pressed) override;
	double get(Wheel wheel) override;
};

///Binarne wejście z podtrzymaniem na zerze
class BinVelsStateHold : public BinVelsState
{
public:
	double get(Wheel wheel) override;
};