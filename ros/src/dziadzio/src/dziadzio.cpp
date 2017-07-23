#include <iostream>
#include <string>
#include <list>
#include <ros/ros.h>
#include <ros/console.h>
#include <omnivelma_msgs/Vels.h>
#include <omnivelma_msgs/EncodersStamped.h>

///Nadajnik Vels
ros::Publisher publisher;

///Funkcja wywoływana na odbiór wiadomości
void encodersCallback(const omnivelma_msgs::EncodersStamped::ConstPtr& msg)
{
	omnivelma_msgs::Vels vels;
	vels.rr = msg -> encoders.velocity.rr;
	vels.rl = msg -> encoders.velocity.rl;
	vels.fr = msg -> encoders.velocity.fr;
	vels.fl = msg -> encoders.velocity.fl;
	publisher.publish(vels);
}

int main(int argc, char** argv)
{
	//ros::init() modyfikuje argumenty
	ros::init(argc, argv, "dziadzio");

	std::list<std::string> topicList;
	if(std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h" || argc != 3)
	{
		//pisz pomoc i wyjdź
		std::cout << "Użycie: " << argv[0] << " INTOPIC OUTTOPIC" << std::endl;
		std::cout << "INTOPIC typu omnivelma_msgs/EncodersStamped." << std::endl;
		std::cout << "OUTTOPIC typu omnivelma_msgs/Vels." << std::endl;
		std::cout << "Program wyłuskuje encoders/velocity i nadaje ponownie." << std::endl;
		return 0;
	}
	std::string inTopic(argv[1]);
	std::string outTopic(argv[2]);

	ros::NodeHandle handle;

	//odbiornik
	ros::Subscriber sub = handle.subscribe<omnivelma_msgs::EncodersStamped>(inTopic, 1, encodersCallback);
	if(!sub)
	{
		ROS_FATAL_STREAM("Nie udało się stworzyć odbiornika " << inTopic);
	}

	//nadajnik
	publisher = handle.advertise<omnivelma_msgs::Vels>(outTopic, 1);
	if(!publisher)
	{
		ROS_FATAL_STREAM("Nie udało się stworzyć nadajnika " << outTopic);
	}

	ros::spin();
	return 0;
}
