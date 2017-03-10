#include <iostream>
#include <ros/ros.h>
#include <omnivelma/Vels.h>
#include <geometry_msgs/Twist.h>

ros::Publisher publisher;

void twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    omnivelma::Vels vels;
	//TODO zmienić
    vels.rr = -1;
    vels.rl = 1;
    vels.fr = -1;
    vels.fl = 1;
    publisher.publish(vels);
}

int main(int argc, char **argv)
{
	std::cout << "Transmutator włączony" << std::endl;
    if (!ros::isInitialized())
    {
		ros::init(argc, argv, "transmutator");
		std::cout << "ROS initializowany w Transmutatorze" << std::endl;
    }

    ros::NodeHandle handle;
    ros::Subscriber sub = handle.subscribe<geometry_msgs::Twist>("/transmutator/twist", 1, twistCallback);
	std::cout << "Zarejestorwano odbiór" << std::endl;
    publisher = handle.advertise<omnivelma::Vels>("/omnivelma/vels", 1);
	std::cout << "Zarejestorwano nadawanie" << std::endl;

    std::cout << "Transmutowanie do Omnivelmy... " << std::endl;
    ros::spin();
	std::cout << "Wychodzenie z Transmutatora" << std::endl;
    return 0;
}
