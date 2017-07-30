#include <functional>
#include <string>
#include <iostream>
#include <cmath>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <ros/console.h>

#define MODEL_NAME std::string("monokl")
#define CLIENT_NAME "gazebo_ros"

namespace gazebo
{
///Klasa sterująca platformą
class Monokl : public ModelPlugin
{
public:
	Monokl()
	{
		counter = 0;
	}

public:
	///Uruchamiane na inicjalizację
	void Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
	{
		model = parent;

		updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&Monokl::OnUpdate, this));

		linkPrefix = std::string(model -> GetName()).append("::").append(MODEL_NAME).append("::");
		std::string topicPrefix = std::string("/").append(model -> GetName()).append("/");

		//inicjalizacja ROSa
		if (!ros::isInitialized())
		{
			int argc = 0;
			char **argv = nullptr;
			ros::init(argc, argv, CLIENT_NAME, ros::init_options::NoSigintHandler);
		}

		//stwórz Node dla ROSa
		rosNode.reset(new ros::NodeHandle());

		
	}


private:
	///Funkcja podłączana do zdarzenia aktualizacji
	void OnUpdate()
	{
		
	}

	///Wskaźnik na model
	physics::ModelPtr model;

	///Wskaźnik na zdarzenie aktualizacji
	event::ConnectionPtr updateConnection;

	///Przedrostek modelu
	std::string linkPrefix;

	///Node dla ROSa
	std::unique_ptr<ros::NodeHandle> rosNode;

	///Licznik kroków
	unsigned int counter;

};

//zarejestruj wtyczkę
GZ_REGISTER_MODEL_PLUGIN(Monokl)
}
