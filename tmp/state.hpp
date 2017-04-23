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
	
	///Ustaw bezpośredie wartości ruchu po płaszczyźnie
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
};

///Stan w którym sterujemy prędkościami kół
class VelsState : public State
{
	
};

///Stan w którym sterujemy kierunkiem
class TwistState : public State
{
	
};