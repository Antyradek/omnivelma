#include <iostream>
#include <string>
#include <ros/ros.h>
#include <omnivelma_msgs/Vels.h>
#include <geometry_msgs/Twist.h>

/// Nadajnik do Omnivelmy
ros::Publisher omniPublisher;
/// Nadajnik do Pseudovelmy
ros::Publisher pseudoPublisher;

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
    vels.rr = (velForw + velRight + (modelLength + modelWidth) * rotLeft * 2) / wheelRadius;
    vels.rl = (velForw - velRight - (modelLength + modelWidth) * rotLeft * 2) / wheelRadius;
    vels.fr = (velForw - velRight + (modelLength + modelWidth) * rotLeft * 2) / wheelRadius;
    vels.fl = (velForw + velRight - (modelLength + modelWidth) * rotLeft * 2) / wheelRadius;
    omniPublisher.publish(vels);
    pseudoPublisher.publish(vels);
}

int main(int argc, char **argv)
{
    rotation = Rotation::No;
    for(int i = 0; i < argc; i++)
    {
        std::string arg(argv[i]);
        if(arg == "--help" || arg == "-h")
        {
            //pisz pomoc i wyjdź
            std::cout << "Użycie:\ntransmutator -90\t\tObróć wejście o 90 stopni w lewo\ntransmutator -180\t\tObróć wejście do tyłu\ntransmutator -270\t\tObróć wejście 90 stopni w prawo" << std::endl;
            return 0;
        }
        else if(arg == "-90")
        {
            rotation = Rotation::Left90;
        }
        else if(arg == "-180")
        {
            rotation = Rotation::Back;
        }
        else if(arg == "-270")
        {
            rotation = Rotation::Right90;
        }
    }
    if (!ros::isInitialized())
    {
        ros::init(argc, argv, "transmutator");
        std::cout << "ROS initializowany w Transmutatorze" << std::endl;
    }

    ros::NodeHandle handle;
    ros::Subscriber sub = handle.subscribe<geometry_msgs::Twist>("/transmutator/twist", 1, twistCallback);
    omniPublisher = handle.advertise<omnivelma_msgs::Vels>("/omnivelma/vels", 1);
    pseudoPublisher = handle.advertise<omnivelma_msgs::Vels>("/pseudovelma/vels", 1);

    std::cout << "Transmutowanie do Omnivelmy... " << std::endl;
    ros::spin();
    std::cout << "Wychodzenie z Transmutatora" << std::endl;
    return 0;
}
