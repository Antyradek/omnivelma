#pragma once
#include "state.hpp"

///Rzeczywiste prędkości ustawiane przytrzymaniem klawiszy
class ContVelsState : public DoubleVelsState
{
protected:
	int w1dir;
	int w2dir;
	int w3dir;
	int w4dir;

	sf::Clock w1clock;
	sf::Clock w2clock;
	sf::Clock w3clock;
	sf::Clock w4clock;

public:
	ContVelsState();
	void update() override;
	void set(sf::Keyboard::Key key, bool pressed) override;
	void reset() override;
	double clamp(double value) const;
};
