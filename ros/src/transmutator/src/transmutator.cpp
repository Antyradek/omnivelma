#include <iostream>
#include <string>
#include <ros/ros.h>
#include <ros/console.h>
#include <omnivelma_msgs/Vels.h>
#include <geometry_msgs/Twist.h>

/// Nadajnik
ros::Publisher publisher;

const double wheelRadius = 0.1;
const double modelWidth = 0.76;
const double modelLength = 0.72;

///Tryb rotacji wejścia
enum Rotation
{
	No,
	Left90,
	Back,
	Right90
};

Rotation rotation;

///Funkcja wywoływana na odbiór wiadomości
void twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	double velForw;
	double velRight;
	double rotLeft = msg -> angular.z;
	switch(rotation)
	{
	case No:
		velForw = msg -> linear.y;
		velRight = msg -> linear.x;
		break;
	case Left90:
		velForw = msg -> linear.x;
		velRight = -msg -> linear.y;
		break;
	case Back:
		velForw = -msg -> linear.y;
		velRight = -msg -> linear.x;
		break;
	case Right90:
		velForw = -msg -> linear.x;
		velRight = msg -> linear.y;
		break;
	}

	omnivelma_msgs::Vels vels;
	vels.fr = (velForw - velRight + (modelLength + modelWidth) * rotLeft * 2) / wheelRadius;
	vels.fl = (velForw + velRight - (modelLength + modelWidth) * rotLeft * 2) / wheelRadius;
	vels.rl = (velForw - velRight - (modelLength + modelWidth) * rotLeft * 2) / wheelRadius;
	vels.rr = (velForw + velRight + (modelLength + modelWidth) * rotLeft * 2) / wheelRadius;
	publisher.publish(vels);
}

///Wypisz pomoc
void printHelp(const std::string& progName)
{
	std::cout << "Użycie: " << progName << " [-r OBRÓT] TOPIC\nPrzekształć wiadomość geometry_msgs/Twist na omnivelma_msgs/Vels obracając wektory wokół osi Z i wyślij do TOPIC.\n" << std::endl;
	std::cout << "  OBRÓT jest jednym z: 0, 90, 180, 270 i obraca wejście o tą ilość stopni w lewo." << std::endl;
	std::cout << "  -h, --help\tWypisz ten ekran." << std::endl;
}

int main(int argc, char **argv)
{
	//ros::init() modyfikuje argumenty
	ros::init(argc, argv, "transmutator");

	std::string progName(argv[0]);
	if(argc <= 1)
	{
		std::cerr << "Nie podano Topica do wysyłania. Wpisz '" << progName << " --help' po instrukcje użycia." << std::endl;
		return -1;
	}
	rotation = Rotation::No;
	std::string topicName;
	int i = 1;
	while (i < argc)
	{
		std::string arg(argv[i]);
		if(arg == "--help" || arg == "-h")
		{
			//pisz pomoc i wyjdź
			printHelp(progName);
			return 0;
		}
		else if(arg == "-r")
		{
			i++;
			if(i < argc)
			{
				std::string rot(argv[i]);
				if(rot == "90")
					rotation = Rotation::Left90;
				else if(rot == "180")
					rotation = Rotation::Back;
				else if(rot == "270")
					rotation = Rotation::Right90;
				else if(rot == "0")
					rotation = Rotation::No;
				else
				{
					std::cerr << "Podano nieprawidłową rotację." << std::endl;
					return -1;
				}
			}
			else
			{
				std::cerr << "Nie podano rotacji." << std::endl;
				return -1;
			}
		}
		else
		{
			topicName = arg;
		}
		i++;
	}
	if(topicName.empty())
	{
		std::cerr << "Nie podano argumentu TOPIC." << std::endl;
		return -1;
	}

	ros::NodeHandle handle;
	ros::Subscriber sub = handle.subscribe<geometry_msgs::Twist>("/transmutator/twist", 1, twistCallback);
	if(!sub)
	{
		ROS_FATAL("Nie można stworzyć odbiornika /transmutator/twist");
	}
	publisher = handle.advertise<omnivelma_msgs::Vels>(topicName, 1);
	if(!publisher)
	{
		ROS_FATAL_STREAM("Nie można stworzyć nadajnika " << topicName);
		return -2;
	}

	ros::spin();
	return 0;
}
