#include <iostream>
#include <string>
#include <list>
#include <ros/ros.h>
#include <ros/console.h>
#include <omnivelma_msgs/Vels.h>

///Lista nadajników
std::list<ros::Publisher> pubList;

///Funkcja wywoływana na odbiór wiadomości
void velsCallback(const omnivelma_msgs::Vels::ConstPtr& msg)
{
	omnivelma_msgs::Vels vels;
	vels.rr = msg -> rr;
	vels.rl = msg -> rl;
	vels.fr = msg -> fr;
	vels.fl = msg -> fl;
	for(ros::Publisher publisher : pubList)
	{
		publisher.publish(vels);
	}
}

int main(int argc, char** argv)
{
	//ros::init() modyfikuje argumenty
	ros::init(argc, argv, "widelnica");

	std::list<std::string> topicList;
	for(int i = 1; i < argc; i++)
	{
		std::string arg(argv[i]);
		if(arg == "--help" || arg == "-h")
		{
			//pisz pomoc i wyjdź
			std::cout << "Użycie: " << argv[0] << " [TOPIC1] [TOPIC2] [TOPIC3]..." << std::endl;
			std::cout << "Rozdziela wejście /widelnica/vels na poszczególne podane TOPIC." << std::endl;
			return 0;
		}
		else
		{
			//dodaj argument do listy
			topicList.push_back(arg);
		}
	}

	ros::NodeHandle handle;
	//odbiornik wiadomości
	ros::Subscriber sub = handle.subscribe<omnivelma_msgs::Vels>("/widelnica/vels", 1, velsCallback);
	if(!sub)
	{
		ROS_FATAL("Nie udało się stworzyć odbiornika /widelnica/vels");
	}

	//lista nadajników
	for(std::string topicName : topicList)
	{
		ros::Publisher newPub = handle.advertise<omnivelma_msgs::Vels>(topicName, 1);
		if(newPub)
		{
			pubList.push_back(newPub);
		}
		else
		{
			ROS_ERROR_STREAM("Nie udało się stworzyć nadajnika " << topicName << " i został on pominięty");
		}
	}

	ros::spin();
	return 0;
}
