#pragma once
#include "state.hpp"
 
class BinVelsState : public VelsState
{
public:
	BinVelsState();
	
private:
	double w1;
	double w2;
	double w3;
	double w4;

	void setDef(sf::Keyboard::Key key, bool pressed) override;
	double getDef(int wheel) override;
	void resetDef() override;
};