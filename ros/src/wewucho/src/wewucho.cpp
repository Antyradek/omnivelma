#include <functional>
#include <string>
#include <iostream>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>

#define CLIENT_NAME "gazebo_ros"

namespace gazebo
{	
///Klasa czujnika inercji
class Wewucho : public SensorPlugin
{
public:
	Wewucho()
	{
		counter = 0;
	}
	
	///Uruchamiane na inicjalizację
	void Load(sensors::SensorPtr parent, sdf::ElementPtr sdf)
	{
		//wyciągnij wskaźnik na odpowiedni czujnik
		sensor = std::dynamic_pointer_cast<sensors::ImuSensor>(parent);
		if(!sensor)
		{
			ROS_FATAL_STREAM("Błąd znajdywania czujnika inercji");
		}
		
		//inicjalizacja ROSa
		if (!ros::isInitialized())
		{
			int argc = 0;
			char **argv = nullptr;
			ros::init(argc, argv, CLIENT_NAME, ros::init_options::NoSigintHandler);
		}
		
		//stwórz Node dla ROSa
		rosNode.reset(new ros::NodeHandle());

		//podłącz zdarzenie aktualizacji
		updateConnection = sensor -> ConnectUpdated(std::bind(&Wewucho::OnUpdate, this));

		//aktywuj sensor
		sensor -> SetActive(true);
		
		//wystaw interfejs ROSa
		publisher = rosNode -> advertise<sensor_msgs::Imu>("/wewucho/imu", 1000);
		if(!publisher)
		{
			ROS_FATAL_STREAM("Nie udało się stworzyć nadajnika /wewucho/imu");
		}

		//powiadom o gotowości
		ROS_DEBUG_STREAM("Wewucho zainicjalizowane");
	}


private:
	///Funkcja podłączana do zdarzenia aktualizacji
	void OnUpdate()
	{
		sensor_msgs::Imu imu;
		
		//uzupełnij nagłówek
		imu.header.seq = counter;
		imu.header.stamp.sec = sensor -> LastMeasurementTime().sec;
		imu.header.stamp.nsec = sensor -> LastMeasurementTime().nsec;
		imu.header.frame_id = "omnivelma";
		
		//uzupełnij parametry czujnika
		imu.orientation.x = sensor -> Orientation().X();
		imu.orientation.y = sensor -> Orientation().Y();
		imu.orientation.z = sensor -> Orientation().Z();
		imu.orientation.w = sensor -> Orientation().W();
		imu.angular_velocity.x = sensor -> AngularVelocity().X();
		imu.angular_velocity.y = sensor -> AngularVelocity().Y();
		imu.angular_velocity.z = sensor -> AngularVelocity().Z();
		imu.linear_acceleration.x = sensor -> LinearAcceleration().X();
		imu.linear_acceleration.y = sensor -> LinearAcceleration().Y();
		imu.linear_acceleration.z = sensor -> LinearAcceleration().Z();
		
		publisher.publish(imu);
		counter++;
	}
	
	///Topic do nadawania pomiarów
	ros::Publisher publisher;
	
	///Wskaźnik na czujnik
	sensors::ImuSensorPtr sensor;
	
	///Licznik kroków
	unsigned int counter;
	
	///Wskaźnik na zdarzenie aktualizacji
	event::ConnectionPtr updateConnection;
	
	///Node dla ROSa
	std::unique_ptr<ros::NodeHandle> rosNode;
};

GZ_REGISTER_SENSOR_PLUGIN(Wewucho)
}


