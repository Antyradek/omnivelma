#pragma once
#include "state.hpp"

///Rzeczywisty kierunek ustawiany kursorem myszy, kółkiem, lub klawiszami
class MouseTwistState : public DoubleTwistState
{
protected:
	int zDir;
	sf::Clock zClock;
	
public:
	MouseTwistState();
	void set(double x, double y) override;
	void set(double deltaZ) override;
	void set(sf::Keyboard::Key key, bool pressed) override;
	void reset() override;
	void update() override;
	double clamp(double value) const;
};
