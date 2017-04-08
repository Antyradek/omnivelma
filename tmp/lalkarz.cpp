#include "lalkarz.hpp"
#include "state.hpp"
#include "bin_vels_state.hpp"
#include "font.hpp"
#include "mono_font.hpp"

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

///Font tekstu
sf::Font font;
sf::Font monoFont;

///Stan robota
std::unique_ptr<VelsState> velsState;

///Pętla drugiego wątku do wysyłania wiadomości ROSa
void sendLoop()
{
	//inicjalizuj ROSa
	while(isActive)
	{
		//zbierz dane i wyślij ROSa
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
	
	//wskazówka do sterowania
	sf::Text modeHelpText(helperText);
	modeHelpText.setString(KEY_TEXT_NEXT_MODE);
	if(showsJoystick)
	{
		modeHelpText.setString(JS_BUTTON_TEXT_NEXT_MODE);
	}
	modeHelpText.setPosition(screenSize - modeHelpText.getGlobalBounds().width, 0);
	window.draw(modeHelpText);
	
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
	
	//Wartości prędkości
	if(sendsVels)
	{
		sf::Text wheelValue(defaultText);
		
		std::stringstream ss;
		ss << std::fixed << std::setprecision(VALUE_PRECISION) << std::right << velsState -> get(2);
		wheelValue.setString(ss.str());
		wheelValue.setPosition((0.25 - VALUE_WHEEL_DIST) * screenSize - wheelValue.getGlobalBounds().width, screenSize * (0.25 + 0.5 * WHEEL_HEIGHT) - wheelValue.getGlobalBounds().height);
		window.draw(wheelValue);
		
		ss.str(std::string());
		ss << std::fixed << std::setprecision(VALUE_PRECISION) << std::right << velsState -> get(3);
		wheelValue.setString(ss.str());
		wheelValue.setPosition((0.25 - VALUE_WHEEL_DIST) * screenSize - wheelValue.getGlobalBounds().width, screenSize * (0.75 - 0.5 * WHEEL_HEIGHT) - wheelValue.getGlobalBounds().height);
		window.draw(wheelValue);
		
		ss.str(std::string());
		ss << std::fixed << std::setprecision(VALUE_PRECISION) << std::right << velsState -> get(1);
		wheelValue.setString(ss.str());
		wheelValue.setPosition((0.75 + VALUE_WHEEL_DIST) * screenSize, screenSize * (0.25 + 0.5 * WHEEL_HEIGHT) - wheelValue.getGlobalBounds().height);
		window.draw(wheelValue);
		
		ss.str(std::string());
		ss << std::fixed << std::setprecision(VALUE_PRECISION) << std::right << velsState -> get(4);
		wheelValue.setString(ss.str());
		wheelValue.setPosition((0.75 + VALUE_WHEEL_DIST) * screenSize, screenSize * (0.75 - 0.5 * WHEEL_HEIGHT) - wheelValue.getGlobalBounds().height);
		window.draw(wheelValue);
	}
	
	//Markery
	if(sendsVels)
	{
		sf::RectangleShape meter(sf::Vector2f(0,0));
		sf::RectangleShape maxMeter1(sf::Vector2f(0,0));
		maxMeter1.setFillColor(sf::Color::Transparent);
		maxMeter1.setOutlineColor(sf::Color::White);
		maxMeter1.setOutlineThickness(HELPER_TEXT_OUTLINE * screenSize);
		sf::RectangleShape maxMeter2(maxMeter1);
		maxMeter1.setSize(sf::Vector2f(METER_WIDTH * screenSize, -METER_HEIGHT * screenSize));
		maxMeter2.setSize(sf::Vector2f(METER_WIDTH * screenSize, METER_HEIGHT * screenSize));
		
		meter.setSize(sf::Vector2f(METER_WIDTH * screenSize, METER_HEIGHT * -velsState -> get(2) * screenSize));
		meter.setPosition((0.25 - METER_WIDTH - METER_WHEEL_DIST) * screenSize, (0.25 + 0.5 * WHEEL_HEIGHT) * screenSize);
		maxMeter1.setPosition(meter.getPosition());
		maxMeter2.setPosition(meter.getPosition());
		window.draw(maxMeter1);
		window.draw(maxMeter2);
		window.draw(meter);
		
		meter.setSize(sf::Vector2f(METER_WIDTH * screenSize, METER_HEIGHT * -velsState -> get(3) * screenSize));
		meter.setPosition((0.25 - METER_WIDTH - METER_WHEEL_DIST) * screenSize, (0.75 - 0.5 * WHEEL_HEIGHT) * screenSize);
		maxMeter1.setPosition(meter.getPosition());
		maxMeter2.setPosition(meter.getPosition());
		window.draw(maxMeter1);
		window.draw(maxMeter2);
		window.draw(meter);
		
		meter.setSize(sf::Vector2f(METER_WIDTH * screenSize, METER_HEIGHT * -velsState -> get(1) * screenSize));
		meter.setPosition((0.75 + METER_WHEEL_DIST) * screenSize, (0.25 + 0.5 * WHEEL_HEIGHT) * screenSize);
		maxMeter1.setPosition(meter.getPosition());
		maxMeter2.setPosition(meter.getPosition());
		window.draw(maxMeter1);
		window.draw(maxMeter2);
		window.draw(meter);
		
		meter.setSize(sf::Vector2f(METER_WIDTH * screenSize, METER_HEIGHT * -velsState -> get(4) * screenSize));
		meter.setPosition((0.75 + METER_WHEEL_DIST) * screenSize, (0.75 - 0.5 * WHEEL_HEIGHT) * screenSize);
		maxMeter1.setPosition(meter.getPosition());
		maxMeter2.setPosition(meter.getPosition());
		window.draw(maxMeter1);
		window.draw(maxMeter2);
		window.draw(meter);
	}
}

///Ustaw następny możliwy tryb
void switchNextMode()
{
	do
	{
		mode = (mode + 1) % MODE_COUNT;
	}while(!enabledModes[mode]);

	showsJoystick = (mode == 5 || mode == 9);
	binaryInput = (mode == 0 || mode == 1 || mode == 6);
	keyWheelInput = (mode == 0 || mode == 1 || mode == 2 || mode == 3 || mode == 4);
	
	switch(mode)
	{
		case 0:
			velsState.reset(new BinVelsState());
			break;
	}
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
	binaryInput = true;
	keyWheelInput = true;
	velsState.reset(new BinVelsState());
	
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
	
	//uruchom program
	std::thread sendThread(sendLoop);
	
    window.create(sf::VideoMode(screenSize, screenSize), "Lalkarz", sf::Style::Close);
	///window.setVerticalSyncEnabled(true);
	window.setFramerateLimit(60);

    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if(event.type == sf::Event::Closed)
			{
				window.close();
			}
			else if((event.type == sf::Event::JoystickButtonPressed && event.joystickButton.button == JS_BUTTON_NEXT_MODE) || (event.type == sf::Event::KeyPressed && event.key.code == KEY_NEXT_MODE))
			{
				switchNextMode();
			}
			else if(event.type == sf::Event::KeyPressed && event.key.code == KEY_STOP)
			{
				velsState -> reset();
			}
			else if(event.type == sf::Event::KeyPressed || event.type == sf::Event::KeyReleased)
			{
				velsState -> set(event.key.code, (event.type == sf::Event::KeyPressed));
			}
			
		}

        window.clear();
        drawGUI();
        window.display();
    }
    
    isActive = false;
    sendThread.join();

    return 0;
} 
