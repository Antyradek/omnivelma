#include <iostream>
#include <ros/ros.h>
#include <omnivelma/Vels.h>
#include <geometry_msgs/Twist.h>

/// Nadajnik do Omnivelmy
ros::Publisher omniPublisher;
/// Nadajnik do Pseudovelmy
ros::Publisher pseudoPublisher;
const double wheelRadius = 0.1;
const double modelWidth = 0.76;
const double modelLength = 0.72;

void twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	const double velForw = msg -> linear.y;
	const double velRight = msg -> linear.x;
	const double rotLeft = msg -> angular.z;

    omnivelma::Vels vels;
    vels.rr = (velForw + velRight + (modelLength + modelWidth) * rotLeft * 2) / wheelRadius;
    vels.rl = (velForw - velRight - (modelLength + modelWidth) * rotLeft * 2) / wheelRadius;
    vels.fr = (velForw - velRight + (modelLength + modelWidth) * rotLeft * 2) / wheelRadius;
    vels.fl = (velForw + velRight - (modelLength + modelWidth) * rotLeft * 2) / wheelRadius;
    omniPublisher.publish(vels);
    pseudoPublisher.publish(vels);
}

int main(int argc, char **argv)
{
    if (!ros::isInitialized())
    {
		ros::init(argc, argv, "transmutator");
		std::cout << "ROS initializowany w Transmutatorze" << std::endl;
    }

    ros::NodeHandle handle;
    ros::Subscriber sub = handle.subscribe<geometry_msgs::Twist>("/transmutator/twist", 1, twistCallback);
    omniPublisher = handle.advertise<omnivelma::Vels>("/omnivelma/vels", 1);
    pseudoPublisher = handle.advertise<omnivelma::Vels>("/pseudovelma/vels", 1);

    std::cout << "Transmutowanie do Omnivelmy... " << std::endl;
    ros::spin();
	std::cout << "Wychodzenie z Transmutatora" << std::endl;
    return 0;
}
