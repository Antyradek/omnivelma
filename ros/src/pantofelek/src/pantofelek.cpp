#include <iostream>
#include <string>
#include <list>
#include <cmath>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#define MOVE_VELOCITY 0.2
#define DEG90RAD 1.570796
#define CLOSE_DIST 0.6
#define LOOP_RATE 10

///Nadajnik twista
ros::Publisher twistPublisher;

///Kierunek robota
typedef enum
{
	Forward,
	Right,
	Left,
	Backward
} Direction;

///Aktualny kierunek robota
Direction direction;

///Przełącz kierunek, bazując na wysłanych danych
void switchDirection(double angle, double scan)
{
	//która kwadra?
	int part = (int)std::floor(angle / DEG90RAD) % 4;
	if(scan < CLOSE_DIST)
	{
		switch(part)
		{
			case 0:
				//prawa przednia
				if(direction == Direction::Forward)
				{
					direction = Direction::Left;
				}
				else if(direction == Direction::Right)
				{
					direction = Direction::Backward;
				}
				break;
			case 1:
				//lewa przednia
				if(direction == Direction::Forward)
				{
					direction = Direction::Right;
				}
				else if(direction == Direction::Left)
				{
					direction = Direction::Backward;
				}
				break;
			case 2:
				//lewa tylna
				if(direction == Direction::Backward)
				{
					direction = Direction::Right;
				}
				else if(direction == Direction::Left)
				{
					direction = Direction::Forward;
				}
				break;
			case 3:
				//prawa tylna
				if(direction == Direction::Backward)
				{
					direction = Direction::Left;
				}
				else if(direction == Direction::Right)
				{
					direction = Direction::Forward;
				}
				break;
			default:
				break;
		}
	}
}

///Wyślij wiadomość Twist, bazując na kierunku
void sendTwistMessage()
{
	geometry_msgs::Twist twist;
	switch(direction)
	{
		case Forward:
			twist.linear.y = MOVE_VELOCITY;
			break;
		case Right:
			twist.linear.x = MOVE_VELOCITY;
			break;
		case Backward:
			twist.linear.y = -MOVE_VELOCITY;
			break;
		case Left:
			twist.linear.x = -MOVE_VELOCITY;
			break;
	}
	twistPublisher.publish(twist);
}

///Funkcja wywoływana na odbiór wiadomości z lewego czujnika, który obrócony jest o kąt 90° (1.570796 rad)
void scanLeftCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	//ignorujemy fakt, że czujnik nie jest idealnie na środku
	for(unsigned int i = 0; i < msg->ranges.size(); i++)
	{
		//bardziej zaawansowany program powinien korzystać z nadawanej ramki, aby obliczyć rotację
		double angle = msg->angle_min + 5 * DEG90RAD + i * msg->angle_increment;
		float scan = msg->ranges[i];
		switchDirection(angle, scan);
	}
}

///Funkcje wywoływana na odbiór wiadomości z prawego czyjnika, który obrócony jest o kąt -90° (-1.570796 rad)
void scanRightCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	for(unsigned int i = 0; i < msg->ranges.size(); i++)
	{
		double angle = msg->angle_min + 3 * DEG90RAD + i * msg->angle_increment;
		float scan = msg->ranges[i];
		switchDirection(angle, scan);
	}
}

int main(int argc, char** argv)
{
	//ros::init() modyfikuje argumenty
	ros::init(argc, argv, "pantofelek");
	std::list<std::string> scanTopicList;
	std::string twistTopicName;
	std::string scannerRName;
	std::string scannerLName;
	
	if(argc == 4)
	{
		twistTopicName = argv[1];
		scannerLName = argv[2];
		scannerRName = argv[3];
	}
	else
	{
		//pisz pomoc i wyjdź
		std::cout << "Użycie: " << argv[0] << " TWIST SCAN_L SCAN_R" << std::endl;
		std::cout << "Przyjmuje wejścia sensor_msgs/LaserScan i generuje wiadomości geometry_msgs/Twist." << std::endl;
		return -1;
	}
	direction = Direction::Forward;
	ros::NodeHandle handle;
	
	//nadajnik wiadomości
	twistPublisher = handle.advertise<geometry_msgs::Twist>(twistTopicName, 1);
	if(!twistPublisher)
	{
		ROS_FATAL_STREAM("Nie udało się stworzyć nadajnika " << twistTopicName);
	}
	
	//odbiornik lewego
	ros::Subscriber scannerLSubscriber = handle.subscribe<sensor_msgs::LaserScan>(scannerLName, 1, scanLeftCallback);
	
	//odbiornik prawego
	ros::Subscriber scannerRSubscriber = handle.subscribe<sensor_msgs::LaserScan>(scannerRName, 1, scanRightCallback);
	
	if(!scannerLSubscriber || !scannerRSubscriber)
	{
		ROS_FATAL_STREAM("Nie udało się stworzyć odbiorników " << scannerRName << " " << scannerLName);
	}
	
	ros::Rate loopRate(LOOP_RATE);
	while(ros::ok())
	{
		sendTwistMessage();
		ros::spinOnce();
		loopRate.sleep();
	}
	return 0;
}
