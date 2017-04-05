#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Window.hpp>
#include <thread>
#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>
#include <exception>
#include "lalkarz.hpp"
#include "font.hpp"

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

///Font tekstu
sf::Font font;

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
	
	sf::Text defaultText("", font);
	defaultText.setCharacterSize(screenSize * FONT_SIZE);
	
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
	
	//tryb
	sf::Text modeText(defaultText);
	modeText.setString(modeNames[mode]);
	modeText.setFillColor(modeColor[mode]);
	modeText.setPosition(0, screenSize * FONT_SIZE);
	window.draw(modeText);
}

///Ustaw następny możliwy tryb
void switchNextMode()
{
	do
	{
		mode = (mode + 1) % MODE_COUNT;
	}while(!enabledModes[mode]);
}

///Wypisz pomoc 
void printHelp()
{
	std::cout << "Lalkarz - program do ręcznego sterowania robotami poprzez kierunek lub prędkości kół za pomocą klawiatury lub kontrolera. Podłącz kontroler, żeby aktywować sterowanie.\n";
	std::cout << "STEROWANIE:\nZ\tPrzełącz tryb\n";
	std::cout << "ARGUMENTY:\n";
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
	sendsTwist = false;
	sendsVels = false;
	
	for(int i = 0; i < MODE_COUNT; i++)
	{
		enabledModes[i] = false;
	}
	modeColor[0] = sf::Color::Yellow;
	modeColor[1] = sf::Color(255, 100, 0, 255);
	modeColor[2] = sf::Color::Red;
	modeColor[3] = sf::Color::Green;
	modeColor[4] = sf::Color(0, 150, 0, 255);
	modeColor[5] = sf::Color(100, 100, 100, 255);
	modeColor[6] = sf::Color::Blue;
	modeColor[7] = sf::Color(100, 0, 255, 255);
	modeColor[8] = sf::Color(200, 0, 255, 255);
	modeColor[9] = sf::Color(100, 0, 100, 255);
	modeColor[10] = sf::Color(255, 0, 255, 255);
	
	modeNames[0] = L"Binarne prędkości kół \nsterowane klawiaturą numeryczną.";
	modeNames[1] = L"Binarne prędkości kół \nz podtrzymaniem \nsterowane klawiaturą numeryczną.";
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
	bool sendsTwist = false;
	bool sendsVels = false;
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
	//wczytaj czcionkę
	font.loadFromMemory(fontData, fontDataSize);
	
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
            if (event.type == sf::Event::Closed)
			{
				window.close();
			}
			if(event.type == sf::Event::KeyPressed)
			{
				if(event.key.code == KEY_NEXT_MODE)
				{
					switchNextMode();
				}
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
