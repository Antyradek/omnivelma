#include <functional>
#include <string>
#include <iostream>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/math/gzmath.hh>
#include <ros/ros.h>
#include <omnivelma_msgs/RelativeStamped.h>
#include <cmath>

namespace gazebo
{
///Klasa sterująca platformą
class Ocznica : public WorldPlugin
{
public:
	Ocznica()
	{
		counter = 0;
	}

	///Uruchamiane na inicjalizację
	void Load(physics::WorldPtr world, sdf::ElementPtr sdfElement)
	{
		//inicjalizacja ROSa
		if (!ros::isInitialized())
		{
			int argc = 0;
			char **argv = NULL;
			ros::init(argc, argv, "gazebo_ros", ros::init_options::NoSigintHandler);
		}

		//stwórz Node dla ROSa
		rosNode.reset(new ros::NodeHandle());
		isCorrect = true;

		//stwórz topic do nadawania wiadomości
		rosPub = rosNode -> advertise<omnivelma_msgs::RelativeStamped>("/ocznica/relative", 1000);
		if(!rosPub)
		{
			ROS_FATAL("Nie udało się stworzyć nadajnika /ocznica/relative");
		}

		//znajdź modele
		omnivelma = world -> GetModel("omnivelma");
		pseudovelma = world -> GetModel("pseudovelma");
		if(!omnivelma || !pseudovelma)
		{
			ROS_ERROR("Nie udało się znaleźć Omnivelmy lub Pseudovelmy");
			isCorrect = false;
		}

		if(isCorrect)
		{
			//podłączenie do wydarznia aktualizacji
			updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&Ocznica::OnUpdate, this));
		}
		
		//powiadom o gotowości
		ROS_DEBUG_STREAM("Ocznica zainicjalizowana");
	}

private:
	///Funkcja podłączana do zdarzenia aktualizacji
	void OnUpdate()
	{
		const math::Pose& omniPose = omnivelma -> GetWorldPose();
		const math::Pose& pseudoPose = pseudovelma -> GetWorldPose();
		double dist = omniPose.pos.Distance(pseudoPose.pos);
		//to działa jedynie dla małych kątów!
		double angle = sqrt(pow(omniPose.rot.x - pseudoPose.rot.x,2) + pow(omniPose.rot.y - pseudoPose.rot.y,2) + pow(omniPose.rot.z - pseudoPose.rot.z,2) + pow(omniPose.rot.w - pseudoPose.rot.w,2));

		omnivelma_msgs::RelativeStamped msg;
		msg.relative.distance = dist;
		msg.relative.angle = angle;
		msg.header.seq = counter;
		msg.header.stamp = ros::Time::now();
		msg.header.frame_id = "0";
		rosPub.publish(msg);
	}


	///Wskaźnik na zdarzenie aktualizacji
	event::ConnectionPtr updateConnection;

	///Node dla ROSa
	std::unique_ptr<ros::NodeHandle> rosNode;

	///Nadajnik pozycji
	ros::Publisher rosPub;

	///Wskaźnik na model dynamiczny
	physics::ModelPtr omnivelma;

	///wskaźnik na model kinematyczny
	physics::ModelPtr pseudovelma;

	///Czy udało się działać
	bool isCorrect;

	///Licznik kroków
	unsigned int counter;

};

//zarejestruj wtyczkę
GZ_REGISTER_WORLD_PLUGIN(Ocznica)
}
