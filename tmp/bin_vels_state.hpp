#pragma once
#include "state.hpp"
 
///Binarne wejście
class BinVelsState : public VelsState
{
public:
	BinVelsState();
	
protected:
	int w1;
	int w2;
	int w3;
	int w4;

	void set(sf::Keyboard::Key key, bool pressed) override;
	double get(int wheel) override;
	void reset() override;
	void update() override;
};

///Binarne wejście z podtrzymaniem na zero
class BinVelsStateHold : public BinVelsState
{
	double get(int wheel) override;
};