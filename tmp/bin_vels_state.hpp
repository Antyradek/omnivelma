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

	void setDef(sf::Keyboard::Key key, bool pressed) override;
	double getDef(int wheel) override;
	void resetDef() override;
	void updateDef() override;
};

///Binarne wejście z podtrzymaniem na zero
class BinVelsStateHold : public BinVelsState
{
	double getDef(int wheel) override;
};