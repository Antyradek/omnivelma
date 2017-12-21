#include <iostream>
#include <fstream>
#include <queue>
#include <string>
#include <time.h>
#include <stdint.h>
#include <signal.h>
#include <cmath>
#include <unistd.h>
#include <cstdlib>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>

#define NANOSECONDS_IN_SECOND 1000000000
#define CLOCK CLOCK_REALTIME

typedef struct
{
	struct timespec moment;
	double x;
	double y;
	double z;
} Instruction;

///Kolejka danych z pliku
std::queue<Instruction> recordQueue;
///ID licznika asynchronicznego
timer_t asyncTimer;
///Czas startu programu
struct timespec startTime;
///Nadajnik Twista
ros::Publisher twistPublisher;

void mainThreadHandler(union sigval val)
{
	geometry_msgs::Twist twist;
	if(recordQueue.size() > 0)
	{
		Instruction instr = recordQueue.front();
		twist.linear.x = instr.x;
		twist.linear.y = instr.y;
		twist.angular.z = instr.z;
	}
	twistPublisher.publish(twist);
}

void asyncThreadHandler(union sigval val)
{
	recordQueue.pop();
	if(recordQueue.size() == 0)
	{
		mainThreadHandler(val);
		timer_delete(asyncTimer);
		return;
	}
	//nastaw licznik na następny moment
	struct itimerspec nextCall;
	nextCall.it_interval.tv_nsec = 0;
	nextCall.it_interval.tv_sec = 0;
	nextCall.it_value.tv_sec = startTime.tv_sec + recordQueue.front().moment.tv_sec;
	nextCall.it_value.tv_nsec = startTime.tv_nsec + recordQueue.front().moment.tv_nsec;
	nextCall.it_value.tv_sec += nextCall.it_value.tv_nsec / NANOSECONDS_IN_SECOND;
	nextCall.it_value.tv_nsec %= NANOSECONDS_IN_SECOND;
	int ret = timer_settime(asyncTimer, TIMER_ABSTIME, &nextCall, nullptr);
	if(ret != 0)
	{
		ROS_FATAL("Błąd uzbrajania pobocznego licznika");
		exit(-1);
	}
	//wyślij wiadomość
	mainThreadHandler(val);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "gramofon");
	
	//argumenty działania
	std::string filename;
	double step;
	std::string topicName;
	
	if(argc == 4)
	{
		step = atof(argv[1]);
		filename = argv[2];
		topicName = argv[3];
	}
	else
	{
		//pisz pomoc i wyjdź
		std::cout << "Użycie: " << argv[0] << " INTERVAL FILE TOPIC" << std::endl;
		std::cout << "Generuj dla TOPIC wiadomość typu geometry_msgs/Twist, co INTERVAL sekund (zmiennoprzecinkowa)." << std::endl;
		std::cout << "Dane odczytaj z pliku, gdzie w każdej linii:" << std::endl;
		std::cout << "TIME X Y Z" << std::endl;
		std::cout << "TIME    czas generowania wiadomości z podanymi wartościami" << std::endl;
		std::cout << "X       prędkość w osi X" << std::endl;
		std::cout << "Y       prędkość w osi Y" << std::endl;
		std::cout << "Z       prędkość kątowa w osi Z" << std::endl;
		return -1;
	}
	
	//stwórz nadajnik
	ros::NodeHandle handle;
	twistPublisher = handle.advertise<geometry_msgs::Twist>(topicName, 1);
	if(!twistPublisher)
	{
		ROS_FATAL_STREAM("Nie udało się stworzyć nadajnika " << topicName);
		return -1;
	}

	//wczytaj plik do listy
	std::ifstream fileStream(filename);
	uint64_t nanoseconds = 0;
	while(fileStream.good()) 
	{
		Instruction instruction;
		double duration;
		fileStream >> duration >> instruction.x >> instruction.y >> instruction.z;
		if(fileStream.good())
		{
			nanoseconds += duration * NANOSECONDS_IN_SECOND;
			instruction.moment.tv_sec = nanoseconds / NANOSECONDS_IN_SECOND;
			instruction.moment.tv_nsec = nanoseconds % NANOSECONDS_IN_SECOND;
			recordQueue.push(instruction);
		}
		else
		{
			break;
		}
	}
	fileStream.close();
	
	//stwórz główny licznik
	timer_t mainTimer;
	struct sigevent event;
	event.sigev_notify = SIGEV_THREAD;
	event.sigev_notify_function = mainThreadHandler;
	event.sigev_value.sival_ptr = nullptr;
	event.sigev_notify_attributes = nullptr;
	int ret = timer_create(CLOCK, &event, &mainTimer);
	if(ret != 0)
	{
		ROS_FATAL("Błąd tworzenia głównego licznika");
		return -1;
	}
	
	//stwórz licznik poboczny
	event.sigev_notify = SIGEV_THREAD;
	event.sigev_notify_function = asyncThreadHandler;
	event.sigev_value.sival_ptr = nullptr;
	event.sigev_notify_attributes = nullptr;
	ret = timer_create(CLOCK, &event, &asyncTimer);
	if(ret != 0)
	{
		ROS_FATAL("Błąd tworzenia pobocznego licznika");
		return -1;
	}
	
	//uruchom główny licznik
	struct itimerspec timerSpec;
	timerSpec.it_value.tv_sec = std::floor(step);
	timerSpec.it_value.tv_nsec = (uint64_t)(step * NANOSECONDS_IN_SECOND) % NANOSECONDS_IN_SECOND;
	timerSpec.it_interval.tv_sec = timerSpec.it_value.tv_sec;
	timerSpec.it_interval.tv_nsec = timerSpec.it_value.tv_nsec;
	ret = timer_settime(mainTimer, TIMER_ABSTIME, &timerSpec, nullptr);
	if(ret != 0)
	{
		ROS_FATAL("Błąd uzbrajania głównego licznika");
		return -1;
	}
	
	//pobierz aktualny czas
	clock_gettime(CLOCK, &startTime);
	
	//uruchom licznik poboczny
	timerSpec.it_interval.tv_sec = 0;
	timerSpec.it_interval.tv_nsec = 0;
	timerSpec.it_value.tv_sec = startTime.tv_sec + recordQueue.front().moment.tv_sec;
	timerSpec.it_value.tv_nsec = startTime.tv_nsec + recordQueue.front().moment.tv_nsec;
	timerSpec.it_value.tv_sec += timerSpec.it_value.tv_nsec / NANOSECONDS_IN_SECOND;
	timerSpec.it_value.tv_nsec %= NANOSECONDS_IN_SECOND;
	ret = timer_settime(asyncTimer, TIMER_ABSTIME, &timerSpec, nullptr);
	if(ret != 0)
	{
		ROS_FATAL("Błąd uzbrajania pobocznego licznika");
		return -1;
	}
	
	//zawieś proces
	ros::spin();
}
