#pragma once

#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Window.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <exception>
#include <algorithm>
#include <memory>
#include <cmath>

///Domyślna wielkość okna
#define WINDOW_SIZE 				512 
///Szerokość rysunku koła
#define WHEEL_WIDTH 				0.05
///Wysokość ryskunku koła
#define WHEEL_HEIGHT				0.1
///Odległość wartości od koła
#define VALUE_WHEEL_DIST			0.1
///Szerokość miernika
#define METER_WIDTH					0.05
///Wysokość miernika
#define METER_HEIGHT				0.18
///Prędkość zmiany miernika
#define METER_CHANGE_SPEED			0.8
///Odległość miernika od koła
#define METER_WHEEL_DIST			0.01
///Wielkość linii
#define FONT_SIZE 					0.03
///Wielkość znaków
#define CHAR_SIZE					0.025
///Precyzja drukowanych wartości
#define VALUE_PRECISION				2
///Odstęp między elementami listy
#define LIST_WIDTH 					0.05
///Szerokość otoczki tekstu
#define HELPER_TEXT_OUTLINE 		0.001

#define MODE_COUNT 					11
#define GEAR_COUNT 					5
#define DEFAULT_FREQ 				10
///Ilość stopni w sterowaniu schodkowym
#define INPUT_STEP_COUNT			10

#define EXIT_ARG_ERROR 				-1
#define EXIT_FONT_ERROR 			-2

//przełączanie trybów
#define KEY_NEXT_MODE 				sf::Keyboard::Key::Z
#define KEY_TEXT_NEXT_MODE 			"Z"
#define JS_BUTTON_NEXT_MODE 		6
#define JS_BUTTON_NEXT_MODE_ALT		8
#define JS_BUTTON_TEXT_NEXT_MODE 	"SELECT"

//stop
#define KEY_STOP					sf::Keyboard::Key::Space

//sterowanie wejściem prędkości kół
#define KEY_WHEEL_1_UP 				sf::Keyboard::Key::Multiply
#define KEY_TEXT_WHEEL_1_UP 		"*"
#define KEY_WHEEL_1_DOWN 			sf::Keyboard::Key::Numpad9
#define KEY_TEXT_WHEEL_1_DOWN 		"9"

#define KEY_WHEEL_2_UP 				sf::Keyboard::Key::Divide
#define KEY_TEXT_WHEEL_2_UP 		"/"
#define KEY_WHEEL_2_DOWN 			sf::Keyboard::Key::Numpad8
#define KEY_TEXT_WHEEL_2_DOWN 		"8"

#define KEY_WHEEL_3_UP				sf::Keyboard::Key::Numpad5
#define KEY_TEXT_WHEEL_3_UP			"5"
#define KEY_WHEEL_3_DOWN			sf::Keyboard::Key::Numpad2
#define KEY_TEXT_WHEEL_3_DOWN		"2"

#define KEY_WHEEL_4_UP				sf::Keyboard::Key::Numpad6
#define KEY_TEXT_WHEEL_4_UP			"6"
#define KEY_WHEEL_4_DOWN			sf::Keyboard::Key::Numpad3
#define KEY_TEXT_WHEEL_4_DOWN		"3"

//sterowanie wejściem prędkości kół kontrolerem
#define AXIS_WHEEL_1				sf::Joystick::Axis::V 
#define AXIS_WHEEL_1_ALT			sf::Joystick::Axis::R
#define AXIS_WHEEL_2				sf::Joystick::Axis::Y
#define AXIS_WHEEL_3				sf::Joystick::Axis::X
#define AXIS_WHEEL_4				sf::Joystick::Axis::U
#define AXIS_WHEEL_4_ALT			sf::Joystick::Axis::Z

///Koło
enum Wheel
{
	W1,
	W2,
	W3,
	W4
};

///Oś po której porusza się urządzenie
enum Axis
{
	X,
	Y,
	Z
};

///Prędkości kół
typedef struct 
{
	double w1;
	double w2;
	double w3;
	double w4;
} Vels;

///Kieunki
typedef struct
{
	double x;
	double y;
	double z;
} Twist;