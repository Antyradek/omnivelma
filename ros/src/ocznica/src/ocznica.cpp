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
		
		//znajdź argumenty
		if(!sdfElement -> HasElement("target1") || !sdfElement -> HasElement("target2"))
		{
			ROS_FATAL("Brak elementów <target1> i <target2> w elemencie <plugin>");
			return;
		}
		std::string target1 = sdfElement -> GetElement("target1") -> GetValue() -> GetAsString();
		std::string target2 = sdfElement -> GetElement("target2") -> GetValue() -> GetAsString();
				
		//znajdź nazwę
		std::string pluginName = sdfElement -> GetAttribute("name") -> GetAsString();
		std::string topicPrefix = std::string("/").append(pluginName).append("/");

		//stwórz Node dla ROSa
		rosNode.reset(new ros::NodeHandle());

		//stwórz topic do nadawania wiadomości
		rosPub = rosNode -> advertise<omnivelma_msgs::RelativeStamped>(topicPrefix + "relative", 1000);
		if(!rosPub)
		{
			ROS_FATAL_STREAM("Nie udało się stworzyć nadajnika " << topicPrefix << "relative");
			return;
		}

		//znajdź modele
		model1 = world -> GetModel(target1);
		model2 = world -> GetModel(target2);
		if(!model1 || !model2)
		{
			ROS_FATAL_STREAM("Nie udało się znaleźć wskazanych modeli");
			return;
		}

		//podłączenie do wydarznia aktualizacji
		updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&Ocznica::OnUpdate, this));
		
		//powiadom o gotowości
		ROS_DEBUG_STREAM("Ocznica zainicjalizowana");
	}

private:
	///Funkcja podłączana do zdarzenia aktualizacji
	void OnUpdate()
	{
		const math::Pose& model1Pose = model1 -> GetWorldPose();
		const math::Pose& model2Pose = model2 -> GetWorldPose();
		double dist = std::sqrt((model1Pose.pos.x - model2Pose.pos.x) * (model1Pose.pos.x - model2Pose.pos.x) + (model1Pose.pos.y - model2Pose.pos.y) * (model1Pose.pos.y - model2Pose.pos.y));
		
		model1Pose.pos.Distance(model2Pose.pos);
		//to działa jedynie dla małych kątów!
		double angle = model1Pose.rot.GetAsEuler().z - model2Pose.rot.GetAsEuler().z;

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

	///Wskaźnik na model 1
	physics::ModelPtr model1;

	///wskaźnik na model 2
	physics::ModelPtr model2;

	///Licznik kroków
	unsigned int counter;

};

//zarejestruj wtyczkę
GZ_REGISTER_WORLD_PLUGIN(Ocznica)
}
