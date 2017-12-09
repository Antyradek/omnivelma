#include <functional>
#include <string>
#include <iostream>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>

#define CLIENT_NAME "gazebo_ros"
#define MONOKL_R_NAME "monokl_r"
#define MONOKL_L_NAME "monokl_l"

namespace gazebo
{	
///Klasa laserowego sensora
class Monokl : public SensorPlugin
{
public:
	Monokl()
	{
		counter = 0;
	}
	
	///Uruchamiane na inicjalizację
	void Load(sensors::SensorPtr parent, sdf::ElementPtr sdf)
	{
		//wyciągnij wskaźnik na odpowiedni czujnik
		sensor = std::dynamic_pointer_cast<sensors::RaySensor>(parent);
		if(!sensor)
		{
			ROS_FATAL_STREAM("Błąd znajdywania czujnika laserowego");
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
		updateConnection = sensor -> ConnectUpdated(std::bind(&Monokl::OnUpdate, this));

		
		//aktywuj sensor
		sensor -> SetActive(true);
		
		//ustaw przedrostek
		/* W czasie importu modelu w modelu, tracimy referencję na model. Zatem trzeba się 
		 * dowiedzieć o tym, czy jest to prawy, czy lewy czujnik, korzystając jedynie z nazwy. */
		std::string name  = sensor -> ParentName();
		if(name.find(MONOKL_R_NAME) != std::string::npos && name.find(MONOKL_L_NAME) == std::string::npos)
		{
			sensorName = MONOKL_R_NAME;
		}
		else if(name.find(MONOKL_R_NAME) == std::string::npos && name.find(MONOKL_L_NAME) != std::string::npos)
		{
			sensorName = MONOKL_L_NAME;
		}
		else
		{
			ROS_FATAL_STREAM("Błąd określania czujnika w modelu, musi on mieć w ścieżce nazwę " << MONOKL_R_NAME << " lub " << MONOKL_L_NAME);
		}
		
		//wystaw interfejs ROSa
		publisher = rosNode -> advertise<sensor_msgs::LaserScan>("/" + sensorName + "/scan", 1000);
		if(!publisher)
		{
			ROS_FATAL_STREAM("Nie udało się stworzyć nadajnika " << sensorName);
		}

		//powiadom o gotowości
		ROS_DEBUG_STREAM("Monokl (" << sensorName << ") zainicjalizowany");
	}


private:
	///Funkcja podłączana do zdarzenia aktualizacji
	void OnUpdate()
	{
		sensor_msgs::LaserScan scan;
		
		//uzupełnij nagłówek
		scan.header.seq = counter;
		scan.header.stamp.sec = sensor -> LastMeasurementTime().sec;
		scan.header.stamp.nsec = sensor -> LastMeasurementTime().nsec;
		scan.header.frame_id = sensorName + "_heart";
		
		//uzupełnij parametry czujnika
		scan.angle_min = sensor -> AngleMin().Radian();
		scan.angle_max = sensor -> AngleMax().Radian();
		scan.angle_increment = sensor -> AngleResolution();
		scan.time_increment = 0;
		scan.scan_time = 1.0 / sensor -> UpdateRate();
		scan.range_max = sensor -> RangeMax();
		scan.range_min = sensor -> RangeMin();
		
		//uzupełnij próbki (szum jest automatycznie dodany)
		for(int i = 0; i < sensor -> RangeCount(); i++)
		{
			scan.ranges.push_back(sensor -> Range(i));
		}
		
		publisher.publish(scan);
		counter++;
	}
	
	///Topic do nadawania pomiarów
	ros::Publisher publisher;
	
	///Wskaźnik na czujnik
	sensors::RaySensorPtr sensor;
	
	///Licznik kroków
	unsigned int counter;
	
	///Wskaźnik na zdarzenie aktualizacji
	event::ConnectionPtr updateConnection;
	
	///Node dla ROSa
	std::unique_ptr<ros::NodeHandle> rosNode;
	
	///Przedrostek nazwy sensora
	std::string sensorName;

};

GZ_REGISTER_SENSOR_PLUGIN(Monokl)
}


