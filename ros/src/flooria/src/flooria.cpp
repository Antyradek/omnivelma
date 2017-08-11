#include <functional>
#include <string>
#include <iostream>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <ros/console.h>
#include <omnivelma_msgs/SetFriction.h>

#define MODEL_NAME std::string("flooria")
#define CLIENT_NAME "gazebo_ros"

namespace gazebo
{
///Klasa podłogi ze zmiennym tarciem
class Flooria : public ModelPlugin
{
public:
	///Uruchamiane na inicjalizację
	void Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
	{
		model = parent;

		linkPrefix = std::string(model -> GetName()).append("::").append(MODEL_NAME).append("::");
		std::string topicPrefix = std::string("/").append(model -> GetName()).append("/");

		pyramid = model -> GetLink(linkPrefix + "base") -> GetCollision("collision") -> GetSurface() -> FrictionPyramid();

		//inicjalizacja ROSa
		if (!ros::isInitialized())
		{
			int argc = 0;
			char **argv = nullptr;
			ros::init(argc, argv, CLIENT_NAME, ros::init_options::NoSigintHandler);
		}

		//stwórz Node dla ROSa
		rosNode.reset(new ros::NodeHandle());

		//stwórz serwer do ustawiania tarcia
		ros::AdvertiseServiceOptions aso = ros::AdvertiseServiceOptions::create<omnivelma_msgs::SetFriction>(topicPrefix.append("set_friction"), std::bind(&Flooria::SetFriction, this, std::placeholders::_1, std::placeholders::_2), nullptr, nullptr);
		rosSrv = rosNode -> advertiseService(aso);
		if(!rosSrv)
		{
			ROS_FATAL_STREAM("Nie udało się aktywować serwera " << topicPrefix.append("set_friction"));
		}
		
		//powiadom o gotowości
		ROS_DEBUG_STREAM("Flooria zainicjalizowana");
	}

private:
	bool SetFriction(omnivelma_msgs::SetFriction::Request& req, omnivelma_msgs::SetFriction::Response& res)
	{
		pyramid -> SetMuPrimary(req.mu1);
		pyramid -> SetMuSecondary(req.mu2);
		ROS_DEBUG("Ustawiono tarcia podłoża: %lf %lf", req.mu1, req.mu2);
		return true;
	}

private:
	///Wskaźnik na model
	physics::ModelPtr model;

	///Wskaźnik piramidy tarcia
	physics::FrictionPyramidPtr pyramid;

	///Przedrostek modelu
	std::string linkPrefix;

	///Node dla ROSa
	std::unique_ptr<ros::NodeHandle> rosNode;

	///Serwer ustawiania tarcia
	ros::ServiceServer rosSrv;

};

//zarejestruj wtyczkę
GZ_REGISTER_MODEL_PLUGIN(Flooria)
}

