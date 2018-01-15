#include <iostream>
#include <string>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <list>

/// Nadajnik
ros::Publisher publisher;

/// Bufor wiadomomości
std::list<sensor_msgs::Imu> messageBuffer;

unsigned int maxQueueSize;

///Funkcja wywoływana na odbiór wiadomości
void msgCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
	sensor_msgs::Imu newImu = *msg;
	messageBuffer.push_front(newImu);
	
	geometry_msgs::Vector3 accelVect;
	unsigned int bufferSize = 0;
	
	//oblicz średnią dla każdego pola
	for(sensor_msgs::Imu imu : messageBuffer)
	{
		accelVect.x += imu.linear_acceleration.x;
		accelVect.y += imu.linear_acceleration.y;
		accelVect.z += imu.linear_acceleration.z;
		bufferSize++;
	}
	accelVect.x /= bufferSize;
	accelVect.y /= bufferSize;
	accelVect.z /= bufferSize;
	
	//wyślij wiadomość
	newImu.linear_acceleration = accelVect;
	publisher.publish(newImu);
	
	//usuń nadmiar wiadomości
	if(bufferSize > maxQueueSize)
	{
		messageBuffer.pop_back();
	}
}

///Wypisz pomoc
void printHelp(const std::string& progName)
{
	std::cout << "Użycie: " << progName << " TOPIC_IN TOPIC_OUT BUFFER\nZmień wiadomość z pierwszej na drugą, korzystając z bufora do zmiękczenia o rozmarze BUFFER." << std::endl;
}

int main(int argc, char **argv)
{
	//ros::init() modyfikuje argumenty
	ros::init(argc, argv, "odszumiacz");

	std::string progName(argv[0]);
	if(argc != 4)
	{
		printHelp(progName);
		return -1;
	}
	
	std::string inTopic(argv[1]);
	std::string outTopic(argv[2]);
	try
	{
		maxQueueSize = stoi(std::string(argv[3]));
	}
	catch(std::invalid_argument err)
	{
		ROS_FATAL_STREAM("Podany rozmiar bufora nie jest liczbą");
		return -1;
	}

	ros::NodeHandle handle;
	ros::Subscriber sub = handle.subscribe<sensor_msgs::Imu>(inTopic, 1000, msgCallback);
	if(!sub)
	{
		ROS_FATAL_STREAM("Nie można stworzyć odbiornika " << inTopic);
		return -1;
	}
	publisher = handle.advertise<sensor_msgs::Imu>(outTopic, 1);
	if(!publisher)
	{
		ROS_FATAL_STREAM("Nie można stworzyć nadajnika " << outTopic);
		return -1;
	}

	ros::spin();
	return 0;
}
