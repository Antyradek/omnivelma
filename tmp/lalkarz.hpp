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
#include <memory>

///Domyślna wielkość okna
#define WINDOW_SIZE 				512 
///Szerokość rysunku koła
#define WHEEL_WIDTH 				0.05
///Wysokość ryskunku koła
#define WHEEL_HEIGHT				0.1
///Odległość wartości od koła
#define VALUE_WHEEL_DIST			0.1
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
#define INPUT_STEP_COUNT			10

#define EXIT_ARG_ERROR 				-1
#define EXIT_FONT_ERROR 			-2

//przełączanie trybów
#define KEY_NEXT_MODE 				sf::Keyboard::Key::Z
#define KEY_TEXT_NEXT_MODE 			"Z"
#define JS_BUTTON_NEXT_MODE 		6
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

