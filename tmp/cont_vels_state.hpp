#pragma once
#include "state.hpp"

class ContVelsState : public VelsState
{
public:
	ContVelsState();
protected:
	int w1dir;
	int w2dir;
	int w3dir;
	int w4dir;
	
	sf::Clock w1clock;
	sf::Clock w2clock;
	sf::Clock w3clock;
	sf::Clock w4clock;
	
	double w1;
	double w2;
	double w3;
	double w4;
	
	void updateDef() override;
	void setDef(sf::Keyboard::Key key, bool pressed) override;
	double getDef(int wheel) override;
	void resetDef() override;
	
	double clamp(double value) const;
};
