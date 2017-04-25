#include "lalkarz.hpp"
#include "state.hpp"
#include "bin_vels_state.hpp"
#include "cont_vels_state.hpp"
#include "steps_vels_state.hpp"
#include "gamepad_vels_state.hpp"
#include "bin_twist_state.hpp"
#include "cont_twist_state.hpp"
#include "font.hpp"
#include "mono_font.hpp"
#include "icon.hpp"

///Czas między kolejnymi wysłaniami wiadomości ROS
double sendWaitTime;
///Czy ma wysyłać wiadomości
std::atomic<bool> isActive;
///Wielkość ekranu
int screenSize;
///Okno programu
sf::RenderWindow window;
///Tryb działania od 0 do MODE_COUNT
int mode;
///Które tryby mogą być włączone
bool enabledModes[MODE_COUNT];
///Kolory trybów
sf::Color modeColor[MODE_COUNT];
///Topic do wysyłania Twist
std::string twistTopic;
///Topic do wysyłania Vels
std::string velsTopic;
///Bieg (przemnożenie wejścia)
int currGear;
///Nazwy trybów
std::wstring modeNames[MODE_COUNT];
///Wysyła wiadomości Twist
bool sendsTwist;
///Wysyła wiadomości Vels
bool sendsVels;

///Pokazuje interfejs joysticka
bool showsJoystick;
///Tryb binarnego wejścia
bool binaryInput;
///Tryb sterowania kołami z klawiatury
bool keyWheelInput;
///Tryb sterowania kołami ogólnie
bool wheelInput;
///Tryb sterowania kierunkiem
bool twistInput;
///Tryb sterowania kierunkiem z klawiatury
bool keyTwistInput;

///Sekcja krytyczna na prędkościach kół
std::mutex mainMutex;
///Dane synchronizowane między wątkami
Vels vels;
///Dane kierunku do synchronizowania między wątkami
Twist twist;

///Font tekstu
sf::Font font;
sf::Font monoFont;

///Stan robota
std::unique_ptr<State> state;

///Pętla drugiego wątku do wysyłania wiadomości ROSa
void sendLoop()
{
	//inicjalizuj ROSa
	while(isActive)
	{
		mainMutex.lock();
			//wyślij
		mainMutex.unlock();
		std::this_thread::sleep_for (std::chrono::milliseconds((int)(sendWaitTime * 1000)));
	}
	//kończ ROSa
}

///Narysuj GUI
void drawGUI()
{
	//pomocnicze
	sf::Text helperText("", font);
	helperText.setCharacterSize(screenSize * CHAR_SIZE);
	helperText.setFillColor(sf::Color(100,100,100,255));
	helperText.setOutlineColor(sf::Color::White);
	helperText.setOutlineThickness(screenSize * HELPER_TEXT_OUTLINE);
	sf::Text defaultText("", font);
	defaultText.setFillColor(sf::Color::White);
	defaultText.setCharacterSize(screenSize * CHAR_SIZE);
	sf::Text valueText("", monoFont);
	valueText.setFillColor(sf::Color::White);
	valueText.setCharacterSize(screenSize * CHAR_SIZE);
	
	//platforma
	sf::RectangleShape wheel(sf::Vector2f(screenSize * WHEEL_WIDTH, screenSize * WHEEL_HEIGHT));
	wheel.setFillColor(modeColor[mode] * sf::Color(100, 100, 100, 255));
	
	sf::RectangleShape wheel2(wheel);
	wheel2.setPosition(screenSize * 0.25, screenSize * 0.25);
	window.draw(wheel2);
	
	sf::RectangleShape wheel3(wheel);
	wheel3.setPosition(screenSize * 0.25, screenSize * (0.75 - WHEEL_HEIGHT));
	window.draw(wheel3);
	
	sf::RectangleShape wheel1(wheel);
	wheel1.setPosition(screenSize * (0.75 - WHEEL_WIDTH), screenSize * 0.25);
	window.draw(wheel1);
	
	sf::RectangleShape wheel4(wheel);
	wheel4.setPosition(screenSize * (0.75 - WHEEL_WIDTH), screenSize * (0.75 - WHEEL_HEIGHT));
	window.draw(wheel4);
	
	sf::RectangleShape body(sf::Vector2f(screenSize * (0.5 - 2 * WHEEL_WIDTH), screenSize * 0.5));
	body.setFillColor(modeColor[mode]);
	body.setPosition(screenSize * (0.25 + WHEEL_WIDTH), screenSize * 0.25);
	window.draw(body);
	
	//lista trybów
	for(int i = 0; i < MODE_COUNT; i++)
	{
		sf::Text modeDigit(defaultText);
		if(enabledModes[i])
		{
			modeDigit.setFillColor(modeColor[i]);
		}
		else
		{
			modeDigit.setFillColor(sf::Color(100,100,100,255));
		}
		if(i == mode)
		{
			modeDigit.setFillColor(sf::Color::White);
		}
		modeDigit.setString(std::to_string(i + 1));
		modeDigit.setPosition(i * screenSize * LIST_WIDTH, 0);
		window.draw(modeDigit);
	}
	
	//wskazówka do przełączania trybów
	sf::Text modeHelpText(helperText);
	if(showsJoystick)
	{
		modeHelpText.setString(JS_BUTTON_TEXT_NEXT_MODE);
	}
	else
	{
		modeHelpText.setString(KEY_TEXT_NEXT_MODE);
	}
	modeHelpText.setPosition(screenSize - modeHelpText.getGlobalBounds().width, 0);
	window.draw(modeHelpText);
	
	//wskazówka zatrzymania
	sf::Text stopText(defaultText);
	stopText.setString(STOP_TEXT);
	stopText.setPosition(0, screenSize * (1.0 - FONT_SIZE));
	window.draw(stopText);
	sf::Text stopHelpText(helperText);
	if(showsJoystick)
	{
		stopHelpText.setString(JS_BUTTON_TEXT_STOP);
	}
	else
	{
		stopHelpText.setString(KEY_TEXT_STOP);
	}
	stopHelpText.setPosition(stopText.getGlobalBounds().width + screenSize * LIST_WIDTH, screenSize * (1.0 - FONT_SIZE));
	window.draw(stopHelpText);
	
	//opis trybu
	sf::Text modeText(defaultText);
	modeText.setString(modeNames[mode]);
	modeText.setFillColor(modeColor[mode]);
	modeText.setPosition(0, screenSize * FONT_SIZE);
	window.draw(modeText);
	
	//Wskazówki do sterowania kołami
	if(keyWheelInput)
	{
		sf::Text wheelHelper(helperText);
		wheelHelper.setString(KEY_TEXT_WHEEL_2_UP);
		wheelHelper.setPosition(screenSize * 0.25, screenSize * (0.25 - FONT_SIZE));
		window.draw(wheelHelper);
		wheelHelper.setString(KEY_TEXT_WHEEL_2_DOWN);
		wheelHelper.setPosition(screenSize * 0.25, screenSize * (0.25 + WHEEL_HEIGHT));
		window.draw(wheelHelper);
		wheelHelper.setString(KEY_TEXT_WHEEL_1_UP);
		wheelHelper.setPosition(screenSize * 0.75 - wheelHelper.getGlobalBounds().width, screenSize * (0.25 - FONT_SIZE));
		window.draw(wheelHelper);
		wheelHelper.setString(KEY_TEXT_WHEEL_1_DOWN);
		wheelHelper.setPosition(screenSize * 0.75 - wheelHelper.getGlobalBounds().width, screenSize * (0.25 + WHEEL_HEIGHT));
		window.draw(wheelHelper);
		wheelHelper.setString(KEY_TEXT_WHEEL_3_UP);
		wheelHelper.setPosition(screenSize * 0.25, screenSize * (0.75 - WHEEL_HEIGHT - FONT_SIZE));
		window.draw(wheelHelper);
		wheelHelper.setString(KEY_TEXT_WHEEL_3_DOWN);
		wheelHelper.setPosition(screenSize * 0.25, screenSize * 0.75);
		window.draw(wheelHelper);
		wheelHelper.setString(KEY_TEXT_WHEEL_4_UP);
		wheelHelper.setPosition(screenSize * 0.75 - wheelHelper.getGlobalBounds().width, screenSize * (0.75 - WHEEL_HEIGHT - FONT_SIZE));
		window.draw(wheelHelper);
		wheelHelper.setString(KEY_TEXT_WHEEL_4_DOWN);
		wheelHelper.setPosition(screenSize * 0.75 - wheelHelper.getGlobalBounds().width, screenSize * 0.75);
		window.draw(wheelHelper);
	}
	
	//Markery
	if(wheelInput)
	{
		//wartości prędkości
		sf::Text wheelValue(defaultText);
		
		std::stringstream ss;
		ss << std::fixed << std::setprecision(VALUE_PRECISION) << std::right << state -> get(Wheel::W2);
		wheelValue.setString(ss.str());
		wheelValue.setPosition((0.25 - VALUE_WHEEL_DIST) * screenSize - wheelValue.getGlobalBounds().width, screenSize * (0.25 + 0.5 * WHEEL_HEIGHT) - wheelValue.getGlobalBounds().height);
		window.draw(wheelValue);
		
		ss.str(std::string());
		ss << std::fixed << std::setprecision(VALUE_PRECISION) << std::right << state -> get(Wheel::W3);
		wheelValue.setString(ss.str());
		wheelValue.setPosition((0.25 - VALUE_WHEEL_DIST) * screenSize - wheelValue.getGlobalBounds().width, screenSize * (0.75 - 0.5 * WHEEL_HEIGHT) - wheelValue.getGlobalBounds().height);
		window.draw(wheelValue);
		
		ss.str(std::string());
		ss << std::fixed << std::setprecision(VALUE_PRECISION) << std::right << state -> get(Wheel::W1);
		wheelValue.setString(ss.str());
		wheelValue.setPosition((0.75 + VALUE_WHEEL_DIST) * screenSize, screenSize * (0.25 + 0.5 * WHEEL_HEIGHT) - wheelValue.getGlobalBounds().height);
		window.draw(wheelValue);
		
		ss.str(std::string());
		ss << std::fixed << std::setprecision(VALUE_PRECISION) << std::right << state -> get(Wheel::W4);
		wheelValue.setString(ss.str());
		wheelValue.setPosition((0.75 + VALUE_WHEEL_DIST) * screenSize, screenSize * (0.75 - 0.5 * WHEEL_HEIGHT) - wheelValue.getGlobalBounds().height);
		window.draw(wheelValue);

		//słupki
		sf::RectangleShape meter(sf::Vector2f(0,0));
		sf::RectangleShape maxMeter1(sf::Vector2f(0,0));
		maxMeter1.setFillColor(sf::Color::Transparent);
		maxMeter1.setOutlineColor(BORDER_COLOR);
		maxMeter1.setOutlineThickness(HELPER_TEXT_OUTLINE * screenSize);
		sf::RectangleShape maxMeter2(maxMeter1);
		maxMeter1.setSize(sf::Vector2f(METER_WIDTH * screenSize, -METER_HEIGHT * screenSize));
		maxMeter2.setSize(sf::Vector2f(METER_WIDTH * screenSize, METER_HEIGHT * screenSize));
		
		meter.setSize(sf::Vector2f(METER_WIDTH * screenSize, METER_HEIGHT * -state -> get(Wheel::W2) * screenSize));
		meter.setPosition((0.25 - METER_WIDTH - METER_WHEEL_DIST) * screenSize, (0.25 + 0.5 * WHEEL_HEIGHT) * screenSize);
		maxMeter1.setPosition(meter.getPosition());
		maxMeter2.setPosition(meter.getPosition());
		window.draw(maxMeter1);
		window.draw(maxMeter2);
		window.draw(meter);
		
		meter.setSize(sf::Vector2f(METER_WIDTH * screenSize, METER_HEIGHT * -state -> get(Wheel::W3) * screenSize));
		meter.setPosition((0.25 - METER_WIDTH - METER_WHEEL_DIST) * screenSize, (0.75 - 0.5 * WHEEL_HEIGHT) * screenSize);
		maxMeter1.setPosition(meter.getPosition());
		maxMeter2.setPosition(meter.getPosition());
		window.draw(maxMeter1);
		window.draw(maxMeter2);
		window.draw(meter);
		
		meter.setSize(sf::Vector2f(METER_WIDTH * screenSize, METER_HEIGHT * -state -> get(Wheel::W1) * screenSize));
		meter.setPosition((0.75 + METER_WHEEL_DIST) * screenSize, (0.25 + 0.5 * WHEEL_HEIGHT) * screenSize);
		maxMeter1.setPosition(meter.getPosition());
		maxMeter2.setPosition(meter.getPosition());
		window.draw(maxMeter1);
		window.draw(maxMeter2);
		window.draw(meter);
		
		meter.setSize(sf::Vector2f(METER_WIDTH * screenSize, METER_HEIGHT * -state -> get(Wheel::W4) * screenSize));
		meter.setPosition((0.75 + METER_WHEEL_DIST) * screenSize, (0.75 - 0.5 * WHEEL_HEIGHT) * screenSize);
		maxMeter1.setPosition(meter.getPosition());
		maxMeter2.setPosition(meter.getPosition());
		window.draw(maxMeter1);
		window.draw(maxMeter2);
		window.draw(meter);
	}
	
	//Wskazówki do sterowania kierunkiem
	if(keyTwistInput)
	{
		sf::Text axisHelper(helperText);
		axisHelper.setString(KEY_TEXT_AXIS_X_DOWN);
		axisHelper.setPosition(screenSize * 0.25, screenSize * (0.5 - FONT_SIZE * 0.5));
		window.draw(axisHelper);
		axisHelper.setString(KEY_TEXT_AXIS_X_UP);
		axisHelper.setPosition(screenSize * 0.75 - axisHelper.getGlobalBounds().width, screenSize * (0.5 - FONT_SIZE * 0.5));
		window.draw(axisHelper);
		axisHelper.setString(KEY_TEXT_AXIS_Y_UP);
		axisHelper.setPosition(screenSize * 0.5 - axisHelper.getGlobalBounds().width * 0.5, screenSize * (0.25 - WHEEL_WIDTH + FONT_SIZE));
		window.draw(axisHelper);
		axisHelper.setString(KEY_TEXT_AXIS_Y_DOWN);
		axisHelper.setPosition(screenSize * 0.5 - axisHelper.getGlobalBounds().width * 0.5, screenSize * (0.75));
		window.draw(axisHelper);
		axisHelper.setString(KEY_TEXT_AXIS_Z_LEFT);
		axisHelper.setPosition(0.25 * screenSize - axisHelper.getGlobalBounds().width, screenSize * (0.25 - 1.5 * METER_WIDTH - 0.5 * FONT_SIZE));
		window.draw(axisHelper);
		axisHelper.setString(KEY_TEXT_AXIS_Z_RIGHT);
		axisHelper.setPosition(0.75 * screenSize, screenSize * (0.25 - 1.5 * METER_WIDTH - 0.5 * FONT_SIZE));
		window.draw(axisHelper);
	}
	
	//Markery kierunku
	if(twistInput)
	{
		//pole
		sf::RectangleShape vectorArea(sf::Vector2f(screenSize * (0.5 - 2 * WHEEL_WIDTH), screenSize * (0.5 - 2 * WHEEL_WIDTH)));
		vectorArea.setFillColor(sf::Color::Transparent);
		vectorArea.setOutlineColor(sf::Color::White);
		vectorArea.setOutlineThickness(screenSize * HELPER_TEXT_OUTLINE);
		vectorArea.setPosition(screenSize * (0.25 + WHEEL_WIDTH), screenSize * (0.25 + WHEEL_WIDTH));
		window.draw(vectorArea);
		sf::RectangleShape midAxis(sf::Vector2f(screenSize * HELPER_TEXT_OUTLINE, vectorArea.getSize().y));
		midAxis.setFillColor(BORDER_COLOR);
		midAxis.setPosition(screenSize * 0.5 - midAxis.getSize().x * 0.5, screenSize * (0.25 + WHEEL_WIDTH));
		window.draw(midAxis);
		midAxis.setSize(sf::Vector2f(vectorArea.getSize().x, screenSize * HELPER_TEXT_OUTLINE));
		midAxis.setPosition(screenSize * (0.25 + WHEEL_WIDTH), screenSize * 0.5 - midAxis.getSize().y * 0.5);
		window.draw(midAxis);
		//strzałka
		double x = state -> getAxis(Axis::X);
		double y = state -> getAxis(Axis::Y);
		double z = state -> getAxis(Axis::Z);
		double r = std::sqrt(x * x + y * y);
		sf::RectangleShape arrow(sf::Vector2f(r * vectorArea.getSize().x * 0.5, screenSize * ARROW_WIDTH));
		sf::CircleShape niceCircle(screenSize * ARROW_WIDTH * 0.5, 20);
		sf::CircleShape arrowEnd(screenSize * ARROW_HEAD_WIDTH, 3);
		arrowEnd.setFillColor(sf::Color::White);
		arrowEnd.setOrigin(screenSize * ARROW_HEAD_WIDTH, screenSize * ARROW_HEAD_WIDTH);
		arrowEnd.setPosition(screenSize * 0.5 + 0.5 * x * vectorArea.getSize().x, screenSize * 0.5 - 0.5 * y * vectorArea.getSize().y);
		arrow.setFillColor(sf::Color::White);
		arrow.setOrigin(0, arrow.getSize().y * 0.5);
		arrow.setPosition(screenSize * 0.5, screenSize * 0.5);
		niceCircle.setFillColor(sf::Color::White);
		niceCircle.setOrigin(screenSize * 0.5 * ARROW_WIDTH, screenSize * 0.5 * ARROW_WIDTH);
		niceCircle.setPosition(0.5 * screenSize, 0.5 * screenSize);
		double phi = std::atan2(y,x);
		arrow.rotate(-phi * RAD2DEG);
		arrowEnd.rotate(-phi * RAD2DEG - 30);
		window.draw(arrow);
		window.draw(niceCircle);
		if(r > ARROW_EPSILON)
		{
			window.draw(arrowEnd);
		}
		//niceCircle.setPosition(screenSize * 0.5 + 0.5 * x * vectorArea.getSize().x, screenSize * 0.5 - 0.5 * y * vectorArea.getSize().y);
		//window.draw(niceCircle);
		//obrót
		sf::RectangleShape maxMeter(sf::Vector2f(0.25 * screenSize, METER_WIDTH * screenSize));
		maxMeter.setFillColor(sf::Color::Transparent);
		maxMeter.setOutlineColor(BORDER_COLOR);
		maxMeter.setOutlineThickness(HELPER_TEXT_OUTLINE * screenSize);
		maxMeter.setPosition(0.25 * screenSize, screenSize * (0.25 - 2 * METER_WIDTH));
		window.draw(maxMeter);
		maxMeter.setPosition(0.5 * screenSize, screenSize * (0.25 - 2 * METER_WIDTH));
		window.draw(maxMeter);
		sf::RectangleShape rotMeter(sf::Vector2f(z * -0.25 * screenSize, METER_WIDTH * screenSize));
		rotMeter.setFillColor(sf::Color::White);
		rotMeter.setPosition(maxMeter.getPosition());
		window.draw(rotMeter);
	}
}

///Ustaw dane powiązane z trybem
void setModeData()
{
	showsJoystick = (mode == 5 || mode == 9);
	binaryInput = (mode == 0 || mode == 1 || mode == 6);
	keyWheelInput = (mode == 0 || mode == 1 || mode == 2 || mode == 3 || mode == 4);
	wheelInput = (mode == 0 || mode == 1 || mode == 2 || mode == 3 || mode == 4 || mode == 5);
	twistInput = (mode == 6 || mode == 7 || mode == 8 || mode == 9 || mode == 10);
	keyTwistInput = (mode == 6 || mode == 7 || mode == 8);
	
	switch(mode)
	{
		case 0:
			state.reset(new BinVelsState());
			break;
		case 1:
			state.reset(new BinVelsStateHold());
			break;
		case 2:
			state.reset(new ContVelsState());
			break;
		case 3:
			state.reset(new StepsVelsState());
			break;
		case 4:
			state.reset(new StepsVelsStateHold());
			break;
		case 5:
			state.reset(new GamepadVelsState());
			break;
		case 6:
			state.reset(new BinTwistState());
			break;
		case 7:
			state.reset(new ContTwistState());
			break;
			
		default:
			break;
	}
}

///Przestaw na następny możliwy tryb
void switchNextMode()
{
	do
	{
		mode = (mode + 1) % MODE_COUNT;
	}while(!enabledModes[mode]);
	
	setModeData();
}

///Wypisz pomoc 
void printHelp()
{
	std::cout << "Lalkarz - program do ręcznego sterowania robotami poprzez kierunek lub prędkości kół za pomocą klawiatury lub kontrolera. Podłącz kontroler, żeby aktywować sterowanie.\n";
	std::cout << "-t <topic>\t\tWysyłaj zadane prędkości do topica typu geometry_msgs/Twist\n";
	std::cout << "-v <topic>\t\tWysyłaj prędkości kół do topica typu omnivelma/Vels\n";
	std::cout << "-f <częstotliwość>\tCzęstotoliwość wysyłania wiadomości w Hz, domyślnie " << DEFAULT_FREQ << "\n";
	std::cout << "-m <tryb>\t\tRozpocznij w podanym trybie działania\n";
	std::cout << "-g <bieg>\t\tRozpocznij w podanym biegu, od 1 do " << GEAR_COUNT << "\n";
	std::cout << "-w <piksele>\t\tUstaw wielkość okna\n";
	std::cout << "-h --help\t\tWypisz tę instrukcję" << std::endl;
}

int main(int args, char** argv)
{
	//ustaw początkowe wartości
	sendWaitTime = 1.0/DEFAULT_FREQ;
	isActive = true;
	screenSize = WINDOW_SIZE;
	mode = 0;
	currGear = 1;
	setModeData();
	
	for(int i = 0; i < MODE_COUNT; i++)
	{
		enabledModes[i] = false;
	}
	modeColor[0] = sf::Color::Yellow;
	modeColor[1] = sf::Color(255, 100, 0, 255);
	modeColor[2] = sf::Color::Red;
	modeColor[3] = sf::Color::Green;
	modeColor[4] = sf::Color(0, 150, 0, 255);
	modeColor[5] = sf::Color(0, 100, 100, 255);
	modeColor[6] = sf::Color::Blue;
	modeColor[7] = sf::Color(100, 0, 255, 255);
	modeColor[8] = sf::Color(200, 0, 255, 255);
	modeColor[9] = sf::Color(100, 0, 100, 255);
	modeColor[10] = sf::Color(255, 0, 255, 255);
	
	modeNames[0] = L"Binarne prędkości kół \nsterowane klawiaturą numeryczną.";
	modeNames[1] = L"Binarne prędkości kół z podtrzymaniem \nsterowane klawiaturą numeryczną.";
	modeNames[2] = L"Przyrostowe prędkości kół \nsterowane klawiaturą numeryczną.";
	modeNames[3] = L"Przyrostowe schodkowe prędkości kół \nsterowane klawiaturą numeryczną.";
	modeNames[4] = L"Przyrostowe schodkowe prędkości kół z podtrzymaniem \nsterowane klawiaturą numeryczną.";
	modeNames[5] = L"Prędkości kół \nsterowane kontrolerem.";
	modeNames[6] = L"Binarny kierunek \nsterowany klawiaturą.";
	modeNames[7] = L"Przyrostowy kierunek \nsterowany klawiaturą.";
	modeNames[8] = L"Przyrostowy schodkowy kierunek \nsterowany klawiaturą.";
	modeNames[9] = L"Kierunek \nsterowany kontrolerem.";
	modeNames[10] = L"Kierunek \nsterowany myszką.";
	
	//odczytywanie argumentów
	int arg = 1;
	sendsTwist = false;
	sendsVels = false;
	bool setMode = false;
	while(arg < args)
	{
		std::string currArg(argv[arg]);
		
		//wypisz pomoc
		if(currArg == "-h" || currArg == "--help")
		{
			printHelp();
			exit(0);
		}
		
		arg++;
		
		if(arg >= args)
		{
			std::cerr << "Niepoprawne argumenty" << std::endl;
			exit(EXIT_ARG_ERROR);
		}
		
		std::string secArg(argv[arg]);
		
		//Twist
		if(currArg == "-t")
		{
			sendsTwist = true;
			twistTopic = secArg;
			enabledModes[6] = true;
			enabledModes[7] = true;
			enabledModes[8] = true;
			enabledModes[10] = true;
		}
		//Vels
		else if(currArg == "-v")
		{
			sendsVels = true;
			velsTopic = secArg;
			enabledModes[0] = true;
			enabledModes[1] = true;
			enabledModes[2] = true;
			enabledModes[3] = true;
			enabledModes[4] = true;
		}
		//częstotliwość
		else if(currArg == "-f")
		{
			try
			{
				double freq = std::stof(secArg);
				if(freq <= 0)
				{
					std::cerr << "Niepoprawna częstotliwość" << std::endl;
					exit(EXIT_ARG_ERROR);
				}
				sendWaitTime = 1.0 / freq;
			}
			catch(std::exception err)
			{
				std::cerr << "Podana częstotliwość nie jest liczbą" << std::endl;
				exit(EXIT_ARG_ERROR);
			}
		}
		//tryb
		else if(currArg == "-m")
		{
			try
			{
				int mod = std::stoi(secArg);
				if(mod <= 0 || mod > MODE_COUNT)
				{
					std::cerr << "Podany tryb jest poza zasięgiem 1-" << MODE_COUNT << std::endl;
					exit(EXIT_ARG_ERROR);
				}
				mode = mod - 1;
				setModeData();
			}
			catch(std::exception err)
			{
				std::cerr << "Podanay tryb nie jest liczbą" << std::endl;
				exit(EXIT_ARG_ERROR);
			}
			setMode = true;
		}
		//bieg
		else if(currArg == "-g")
		{
			try
			{
				int gear = std::stoi(secArg);
				if(gear <= 0 || gear > GEAR_COUNT)
				{
					std::cerr << "Podany bieg jest poza zasięgiem 1-" << GEAR_COUNT << std::endl;
					exit(EXIT_ARG_ERROR);
				}
				currGear = gear;
			}
			catch(std::exception err)
			{
				std::cerr << "Podany bieg nie jest liczbą" << std::endl;
				exit(EXIT_ARG_ERROR);
			}
		}
		else if(currArg == "-w")
		{
			try
			{
				int size = std::stoi(secArg);
				if(size <= 0)
				{
					std::cerr << "Podana wielkość okna jest ujemna" << std::endl;
					exit(EXIT_ARG_ERROR);
				}
				screenSize = size;
			}
			catch(std::exception err)
			{
				std::cerr << "Podana wialkość okna nie jest liczbą" << std::endl;
				exit(EXIT_ARG_ERROR);
			}
		}
		else
		{
			std::cerr << "Nierozpoznany argument " << currArg << std::endl;
			exit(EXIT_ARG_ERROR);
		}
		
		arg++;
	}
	//sprawdź podłączenie joysticka
	sf::Joystick::update();
	if(sf::Joystick::isConnected(0))
	{
		if(sendsTwist)
		{
			enabledModes[9] = true;
		}
		if(sendsVels)
		{
			enabledModes[5] = true;
		}
	}
	//czy podany tryb może być aktywowany
	if(setMode)
	{
		if(!enabledModes[mode])
		{
			std::cerr << "Podany tryb nie może być aktywowany" << std::endl;
			exit(EXIT_ARG_ERROR);
		}
	}
	//czy podał choć jeden topic
	if(!sendsTwist && !sendsVels)
	{
		std::cerr << "Nie ma gdzie wysyłać wiadomości" << std::endl;
		exit(EXIT_ARG_ERROR);
	}
	//ustaw tryb
	if(!enabledModes[mode])
	{
		switchNextMode();
	}
	//wczytaj czcionki
	font.loadFromMemory(fontData, fontDataSize);
	monoFont.loadFromMemory(monoFontData, monoFontDataSize);
	
	//wczytaj ikonę
	sf::Image icon;
	icon.loadFromMemory(iconData, iconDataSize);
	
	//uruchom program
	std::thread sendThread(sendLoop);
	
    window.create(sf::VideoMode(screenSize, screenSize), "Lalkarz", sf::Style::Close);
	///window.setVerticalSyncEnabled(true);
	window.setFramerateLimit(60);
	window.setIcon(icon.getSize().x, icon.getSize().y, icon.getPixelsPtr());

    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if(event.type == sf::Event::Closed)
			{
				window.close();
			}
			else if((event.type == sf::Event::KeyPressed && event.key.code == KEY_NEXT_MODE) || (event.type == sf::Event::JoystickButtonPressed && (event.joystickButton.button == JS_BUTTON_NEXT_MODE || event.joystickButton.button == JS_BUTTON_NEXT_MODE_ALT)))
			{
				switchNextMode();
			}
			else if((event.type == sf::Event::KeyPressed && event.key.code == KEY_STOP) || (event.type == sf::Event::JoystickButtonPressed && (event.joystickButton.button == JS_BUTTON_STOP || event.joystickButton.button == JS_BUTTON_STOP_ALT)))
			{
				state -> reset();
			}
			else if(event.type == sf::Event::KeyPressed || event.type == sf::Event::KeyReleased)
			{
				state -> set(event.key.code, (event.type == sf::Event::KeyPressed));
			}
			else if(event.type == sf::Event::JoystickMoved)
			{
				state -> set(event.joystickMove.axis, event.joystickMove.position);
			}
		}
		state -> update();

        window.clear();
        drawGUI();
        window.display();
		
		mainMutex.lock();
		vels = state -> getVels();
		mainMutex.unlock();
    }
    
    isActive = false;
    sendThread.join();

    return 0;
} 
