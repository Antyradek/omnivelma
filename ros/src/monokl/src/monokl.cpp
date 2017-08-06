#include <functional>
#include <string>
#include <iostream>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <ros/ros.h>
#include <ros/console.h>

#define CLIENT_NAME "gazebo_ros"

namespace gazebo
{
///Klasa sterująca platformą
class Monokl : public SensorPlugin
{
public:
	Monokl()
	{
		counter = 0;
	}

public:
	///Uruchamiane na inicjalizację
	void Load(sensors::SensorPtr parent, sdf::ElementPtr sdf)
	{
		//wyciągnij wskaźnik na odpowiedni czujnik
		sensor = std::dynamic_pointer_cast<sensors::RaySensor>(parent);
		if(!sensor)
		{
			ROS_FATAL_STREAM("Błąd znajdywania czujnika laserowego");
		}

		//podłącz zdarzenie aktualizacji
		updateConnection = sensor -> ConnectUpdated(std::bind(&Monokl::OnUpdate, this));

		//inicjalizacja ROSa
		if (!ros::isInitialized())
		{
			int argc = 0;
			char **argv = nullptr;
			ros::init(argc, argv, CLIENT_NAME, ros::init_options::NoSigintHandler);
		}

		//stwórz Node dla ROSa
		rosNode.reset(new ros::NodeHandle());
		
		//aktywuj sensor
		sensor -> SetActive(true);

		//powiadom o gotowości
		ROS_DEBUG_STREAM("Monokl (" << sensor -> ScopedName() << ") zainicjalizowany");
		
		std::cout << sensor -> ScopedName() << std::endl;
	}


private:
	///Funkcja podłączana do zdarzenia aktualizacji
	void OnUpdate()
	{
		
	}
	
	///Wskaźnik na zdarzenie aktualizacji
	event::ConnectionPtr updateConnection;

	///Wskaźnik na czujnik
	sensors::RaySensorPtr sensor;
	
	///Node dla ROSa
	std::unique_ptr<ros::NodeHandle> rosNode;

	///Licznik kroków
	unsigned int counter;

};
GZ_REGISTER_SENSOR_PLUGIN(Monokl)
}


