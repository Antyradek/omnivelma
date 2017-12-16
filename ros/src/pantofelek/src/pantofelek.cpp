#include <iostream>
#include <string>
#include <list>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#define MOVE_VELOCITY 0.01

///Lista nadajników
ros::Publisher twistPublisher;

///Funkcja wywoływana na odbiór wiadomości
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	geometry_msgs::Twist twist;
	//log
// 	ROS_INFO("-- Otrzymano skan --");
	for(unsigned int i = 0; i < msg->ranges.size(); i++)
	{
		double angle = msg->angle_min + i * msg->angle_increment;
// 		ROS_INFO_STREAM("[" << angle << "] " << msg->ranges[i]);
	}
	twist.linear.y = MOVE_VELOCITY;
	twistPublisher.publish(twist);
}

int main(int argc, char** argv)
{
	//ros::init() modyfikuje argumenty
	ros::init(argc, argv, "pantofelek");
	std::list<std::string> scanTopicList;
	std::string twistTopicName;
	
	for(int i = 1; i < argc; i++)
	{
		std::string arg(argv[i]);
		if(arg == "--help" || arg == "-h")
		{
			//pisz pomoc i wyjdź
			std::cout << "Użycie: " << argv[0] << " [TWIST] [SCAN] [SCAN]..." << std::endl;
			std::cout << "Przyjmuje wejścia sensor_msgs/LaserScan i generuje wiadomości geometry_msgs/Twist." << std::endl;
			return 0;
		}
		else if(i == 1)
		{
			//zapisz argument twist
			twistTopicName = arg;
		}
		else
		{
			//dodaj argument do listy
			scanTopicList.push_back(arg);
		}
	}
	
	ros::NodeHandle handle;
	//nadajnik wiadomości
	twistPublisher = handle.advertise<geometry_msgs::Twist>(twistTopicName, 1);
	if(!twistPublisher)
	{
		ROS_FATAL_STREAM("Nie udało się stworzyć odbiornika " << twistTopicName);
	}
	
	//lista odbiorników, należy zapamiętać obiekty odbiornika, gdyż na destrukcję automatycznie się odłączają
	std::list<ros::Subscriber> scanSubscriberList;
	for(std::string topicName : scanTopicList)
	{
		//dla uproszczenia, każdy czujnik wywołuje tę samą funkcję
		ros::Subscriber newSub = handle.subscribe<sensor_msgs::LaserScan>(topicName, 1, scanCallback);
		if(!newSub)
		{
			ROS_ERROR_STREAM("Nie udało się stworzyć nadajnika " << topicName << " i został on pominięty");
		}
		else
		{
			scanSubscriberList.push_back(newSub);
		}
	}
	
	ros::spin();
	return 0;
}
