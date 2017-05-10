#pragma once
#include "state.hpp"

class MouseTwistState : public DoubleTwistState
{
public:
	void set(double x, double y) override;
	void set(double deltaZ) override;
	double clamp(double value) const;
};
