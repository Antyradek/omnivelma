#pragma once
#include "lalkarz.hpp"

///Bazowy stan dla wszystkich
class State
{
public:
	///Powiadom o naciśnięciu przycisku klawiatury
	virtual void set(sf::Keyboard::Key key, bool pressed);

	///Powiadom o zmianie osi kontrolera
	virtual void set(sf::Joystick::Axis axis, double position);

	///Ustaw bezpośrednie wartości ruchu po płaszczyźnie
	virtual void set(double x, double y);

	///Ustaw zmianę obrotu wokół osi
	virtual void set(double deltaZ);

	///Resetuj stan do zera
	virtual void reset();

	///Aktualizuj wartości zmienne w czasie
	virtual void update();

	///Zwróć prędkość koła
	virtual double get(Wheel wheel);

	///Zwróć strukturę prędkości
	Vels getVels();

	///Zwróć wartość osi
	virtual double getAxis(Axis axis);

	///Zwróć strukturę kierunku
	Twist getTwist();
	
	///Zwróć wartość biegu
	std::vector<double> getGears();
	
protected:
	///Wartości biegów dla tego trybu
	std::vector<double> gears;
	
};

///Stan w którym sterujemy prędkościami kół
class VelsState : public State
{

};

///Stan w którym sterujemy kierunkiem
class TwistState : public State
{

};

///Stan o prędkościach kół o wartości całkowitej
class IntVelsState : public VelsState
{
protected:
	int w1;
	int w2;
	int w3;
	int w4;

public:
	IntVelsState();
	void reset() override;
};

///Stan kierunku z wektorami o wartości całkowitej
class IntTwistState : public TwistState
{
protected:
	int x;
	int y;
	int z;
public:
	IntTwistState();
	void reset() override;
};

///Stan o prędkościach kół o wartości rzeczywistej
class DoubleVelsState : public VelsState
{
protected:
	double w1;
	double w2;
	double w3;
	double w4;

public:
	DoubleVelsState();
	void reset() override;
	double get(Wheel wheel) override;
};

///Stan kierunku z wektorami o wartości rzeczywistej
class DoubleTwistState : public TwistState
{
protected:
	double x;
	double y;
	double z;

public:
	DoubleTwistState();
	void reset() override;
	double getAxis(Axis axis) override;
};
