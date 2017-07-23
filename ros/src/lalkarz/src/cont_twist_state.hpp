#pragma once
#include "state.hpp"

///Rzeczywisty kierunek ustawiany przytrzymaniem klawiszy
class ContTwistState : public DoubleTwistState
{
protected:
	int xDir;
	int yDir;
	int zDir;

	sf::Clock xClock;
	sf::Clock yClock;
	sf::Clock zClock;

public:
	ContTwistState();
	void update() override;
	void set(sf::Keyboard::Key key, bool pressed) override;
	void reset() override;
	double clamp(double value) const;
};
