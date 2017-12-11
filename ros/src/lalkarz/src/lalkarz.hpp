#pragma once

#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Window.hpp>
#include <ros/ros.h>
#include <omnivelma_msgs/Vels.h>
#include <geometry_msgs/Twist.h>
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
#include <vector>

///Domyślna wielkość okna
#define WINDOW_SIZE 				700
///Szerokość rysunku koła
#define WHEEL_WIDTH 				0.05
///Wysokość rysunku koła
#define WHEEL_HEIGHT				0.1
///Odległość wartości od koła
#define VALUE_WHEEL_DIST			0.13
///Szerokość miernika
#define METER_WIDTH					0.05
///Szerokość miernika enkoderów
#define ENC_METER_WIDTH				0.02
///Wysokość miernika
#define METER_HEIGHT				0.18
///Prędkość zmiany miernika
#define METER_CHANGE_SPEED			1.0
///Odległość miernika od koła
#define METER_WHEEL_DIST			0.05
///Wielkość linii
#define FONT_SIZE 					0.03
///Wielkość znaków
#define CHAR_SIZE					0.025
///Precyzja drukowanych wartości
#define VALUE_PRECISION				3
///Odstęp między elementami listy
#define LIST_WIDTH 					0.08
///Odstęp między elementami listy biegów
#define GEAR_LIST_WIDTH 			0.1
///Szerokość otoczki tekstu
#define HELPER_TEXT_OUTLINE 		0.001
///Szerokość otoczki elementów
#define HELPER_RECT_OUTLINE			0.002
///Szerokość strzałki
#define ARROW_WIDTH					0.01
///Wielkość grotu strzałki
#define ARROW_HEAD_WIDTH			0.02
///Kolor otoczki
#define BORDER_COLOR				sf::Color(255,255,255,100)
///Kolor wyłączenia
#define DISABLED_COLOR				sf::Color(100,100,100,255)
///Minimalna wartość, aby pokazać zmianę strzałki
#define ARROW_EPSILON				0.01

///Ilość trybów
#define MODE_COUNT 					11
///Domyślna częstotliwość
#define DEFAULT_FREQ 				10
///Ilość stopni w sterowaniu schodkowym
#define INPUT_STEP_COUNT			10

#define EXIT_ARG_ERROR 				0x01
#define EXIT_FONT_ERROR 			0x02

//przełączanie trybów
#define KEY_NEXT_MODE 				sf::Keyboard::Key::Z
#define KEY_TEXT_NEXT_MODE 			"Z"
#define JS_BUTTON_NEXT_MODE 		6
#define JS_BUTTON_NEXT_MODE_ALT		8
#define JS_BUTTON_TEXT_NEXT_MODE 	"SELECT"

//przełączanie biegów
#define KEY_GEAR_UP					sf::Keyboard::Key::R
#define KEY_GEAR_DOWN				sf::Keyboard::Key::F
#define KEY_TEXT_GEAR_UP			"R"
#define KEY_TEXT_GEAR_DOWN			"F"
#define KEY_GEAR_UP_ALT				sf::Keyboard::Key::Add
#define KEY_GEAR_DOWN_ALT			sf::Keyboard::Key::Subtract
#define KEY_TEXT_GEAR_UP_ALT		"+"
#define KEY_TEXT_GEAR_DOWN_ALT		"-"
#define JS_BUTTON_GEAR_UP			5
#define JS_BUTTON_GEAR_DOWN			4
#define JS_BUTTON_TEXT_GEAR_UP		"RB"
#define JS_BUTTON_TEXT_GEAR_DOWN	"LB"

//stop
#define KEY_STOP					sf::Keyboard::Key::Space
#define JS_BUTTON_STOP				7
#define JS_BUTTON_STOP_ALT			9
#define STOP_TEXT					"Zatrzymaj"
#define KEY_TEXT_STOP				"SPACJA"
#define JS_BUTTON_TEXT_STOP			"START"

//sterowanie prędkością kół
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

//sterowanie prędkością kół kontrolerem
#define AXIS_WHEEL_1				sf::Joystick::Axis::V
#define AXIS_WHEEL_1_ALT			sf::Joystick::Axis::R
#define AXIS_WHEEL_2				sf::Joystick::Axis::Y
#define AXIS_WHEEL_3				sf::Joystick::Axis::X
#define AXIS_WHEEL_4				sf::Joystick::Axis::U
#define AXIS_WHEEL_4_ALT			sf::Joystick::Axis::Z
#define AXIS_TEXT_HORIZONTAL		L"Poziome osie joysticków"
#define AXIS_TEXT_VERTICAL			L"Pionowe osie joysticków"

//sterowanie kierunkiem kontrolerem
#define AXIS_TWIST_X				sf::Joystick::Axis::U
#define AXIS_TWIST_X_ALT			sf::Joystick::Axis::Z
#define AXIS_TWIST_Y				sf::Joystick::Axis::V
#define AXIS_TWIST_Y_ALT			sf::Joystick::Axis::R
#define AXIS_TWIST_Z				sf::Joystick::Axis::X
#define AXIS_TEXT_TWIST_DIR			"Prawy joystick"
#define AXIS_TEXT_TWIST_ROT			L"Pozioma oś lewego joysticka"

//sterowanie kierunkiem myszką
#define MOUSE_TEXT_POSITION			"Pozycja kursora myszy"
#define MOUSE_TEXT_ROTATION			L"Kółko myszy"

//sterowanie kierunkiem klawiaturą
#define KEY_AXIS_X_UP				sf::Keyboard::Key::D
#define KEY_TEXT_AXIS_X_UP			"D"
#define KEY_AXIS_X_DOWN				sf::Keyboard::Key::A
#define KEY_TEXT_AXIS_X_DOWN		"A"

#define KEY_AXIS_Y_UP				sf::Keyboard::Key::W
#define KEY_TEXT_AXIS_Y_UP			"W"
#define KEY_AXIS_Y_DOWN				sf::Keyboard::Key::S
#define KEY_TEXT_AXIS_Y_DOWN		"S"

#define KEY_AXIS_Z_LEFT				sf::Keyboard::Key::Q
#define KEY_TEXT_AXIS_Z_LEFT		"Q"
#define KEY_AXIS_Z_RIGHT			sf::Keyboard::Key::E
#define KEY_TEXT_AXIS_Z_RIGHT		"E"

//radiany na kąty
#define RAD2DEG 					57.2957795130824

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

//Tryb wysyłania
enum SendMode
{
	SendVels,
	SendTwist
};

///Prędkości kół
typedef struct
{
	double w1;
	double w2;
	double w3;
	double w4;
} Vels;

///Kierunki
typedef struct
{
	double x;
	double y;
	double z;
} Twist;

