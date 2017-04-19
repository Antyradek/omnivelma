#include <iostream>
#include <string>
#include <list>
#include <ros/ros.h>
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

int main(int argc, char **argv)
{
	std::list<std::string> topicList;
    for(int i = 1; i < argc; i++)
    {
        std::string arg(argv[i]);
        if(arg == "--help" || arg == "-h")
        {
            //pisz pomoc i wyjdź
            std::cout << "Użycie:\nwidelnica <topic1> <topic2> <topic3> <...>" << std::endl;
            return 0;
        }
        else
		{
			//dodaj argument do listy
			topicList.push_back(arg);
		}
    }
    if (!ros::isInitialized())
    {
        ros::init(argc, argv, "widelnica");
        std::cout << "ROS initializowany w Widelnicy" << std::endl;
    }

    ros::NodeHandle handle;
	//odbiornik wiadomości
    ros::Subscriber sub = handle.subscribe<omnivelma_msgs::Vels>("/widelnica/vels", 1, velsCallback);
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
			std::cerr << "Nie udało się zainicjalizować wysyłania do " << topicName << " i został on pominięty." << std::endl;
		}
	}

    std::cout << "Transmutowanie do Omnivelmy... " << std::endl;
    ros::spin();
    std::cout << "Wychodzenie z Transmutatora" << std::endl;
    return 0;
}
