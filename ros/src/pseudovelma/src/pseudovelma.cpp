#include <functional>
#include <string>
#include <iostream>
#include <cmath>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/math/gzmath.hh>
#include <ros/ros.h>
#include <ros/console.h>
#include <tf2_ros/transform_broadcaster.h>
#include <omnivelma_msgs/Vels.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

#define MODEL_NAME std::string("pseudovelma")
#define MAP_TF "map"

namespace gazebo
{
///Klasa sterująca platformą
class Pseudovelma : public ModelPlugin
{
public:
	///Konstruktor dba, aby prędkość początkowa była 0
	Pseudovelma()
	{
		//zmienne napisane tak, jak ustawione są koła
		flVel = 0;
		frVel = 0;
		rlVel = 0;
		rrVel = 0;

		//z modelu
		wheelRadius = 0.1;
		modelWidth = 0.76;
		modelLength = 0.72;

		counter = 0;
	}

	///Uruchamiane na inicjalizację
	void Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
	{
		model = parent;

		linkPrefix = std::string(model -> GetName()).append("::").append(MODEL_NAME).append("::");
		std::string topicPrefix = std::string("/").append(model -> GetName()).append("/");

		//podłączenie do wydarznia aktualizacji
		updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&Pseudovelma::OnUpdate, this));

		//inicjalizacja ROSa
		if (!ros::isInitialized())
		{
			int argc = 0;
			char **argv = NULL;
			ros::init(argc, argv, "gazebo_ros", ros::init_options::NoSigintHandler);
		}

		//stwórz Node dla ROSa
		rosNode.reset(new ros::NodeHandle());

		//Stwórz topic do odbierania wiadomości
		rosSub = rosNode -> subscribe<omnivelma_msgs::Vels>(topicPrefix + "vels", 1, std::bind(&Pseudovelma::OnRosMsg, this, std::placeholders::_1));
		if(!rosSub)
		{
			ROS_FATAL_STREAM("Nie udało się ustawić odbiornika " << topicPrefix + "vels");
		}

		//stwórz topic do nadawania pozycji
		rosPose = rosNode -> advertise<geometry_msgs::PoseStamped>(topicPrefix + "pose", 1000);
		if(!rosPose)
		{
			ROS_FATAL_STREAM("Nie udało się ustawić nadajnika " << topicPrefix + "pose");
		}

		//stwórz topic do nadawania prędkości
		rosTwist = rosNode -> advertise<geometry_msgs::TwistStamped>(topicPrefix + "twist", 1000);
		if(!rosTwist)
		{
			ROS_FATAL_STREAM("Nie udało się ustawić nadajnika " << topicPrefix + "twist");
		}
		
		//stwórz topic do nadawania odometrii
		rosOdometry = rosNode -> advertise<nav_msgs::Odometry>(topicPrefix + "odometry", 1000);
		if(!rosOdometry)
		{
			ROS_FATAL_STREAM("Nie udało się ustawić nadajnika " << topicPrefix + "odometry");
		}
		
		//powiadom o gotowości
		ROS_DEBUG_STREAM("Pseudovelma zainicjalizowana");
	}

private:
	///Funkcja podłączana do zdarzenia aktualizacji
	void OnUpdate()
	{
		double velX = -frVel + flVel - rlVel + rrVel;
		double velY = frVel + flVel + rlVel + rrVel;
		velX *= 0.25 * wheelRadius;
		velY *= 0.25 * wheelRadius;
		math::Vector3 transVect = math::Vector3(velX, velY, 0);
		double k = 2.0/(modelWidth + modelLength);
		double rot = frVel - flVel - rlVel + rrVel;
		rot *= k * 0.25 * wheelRadius;

		//przerabianie lokalnych wektorów na globalne
		math::Quaternion modelRot = model -> GetWorldPose().rot;
		transVect = modelRot.RotateVector(transVect);

		model -> SetAngularVel(math::Vector3(0,0,rot));
		model -> SetLinearVel(transVect);

		//TODO obracanie kołami dla ozdoby

		//wyślij pozycję
		const math::Pose& pose = model -> GetWorldPose();
		geometry_msgs::PoseStamped poseMsg;
		poseMsg.pose.position.x = pose.pos.x;
		poseMsg.pose.position.y = pose.pos.y;
		poseMsg.pose.position.z = pose.pos.z;
		poseMsg.pose.orientation.x = pose.rot.x;
		poseMsg.pose.orientation.y = pose.rot.y;
		poseMsg.pose.orientation.z = pose.rot.z;
		poseMsg.pose.orientation.w = pose.rot.w;
		poseMsg.header.seq = counter;
		poseMsg.header.stamp = ros::Time::now();
		poseMsg.header.frame_id = MAP_TF;
		rosPose.publish(poseMsg);

		//wyślij prędkość
		const math::Vector3 linVel = model -> GetWorldLinearVel();
		const math::Vector3 angVel = model -> GetWorldAngularVel();
		geometry_msgs::TwistStamped twistMsg;
		twistMsg.twist.linear.x = linVel.x;
		twistMsg.twist.linear.y = linVel.y;
		twistMsg.twist.linear.z = linVel.z;
		twistMsg.twist.angular.x = angVel.x;
		twistMsg.twist.angular.y = angVel.y;
		twistMsg.twist.angular.z = angVel.z;
		twistMsg.header.seq = counter;
		twistMsg.header.stamp = ros::Time::now();
		twistMsg.header.frame_id = MAP_TF;
		rosTwist.publish(twistMsg);
		
		//wyślij odometrię
		nav_msgs::Odometry odometryMsg;
		odometryMsg.header.seq = counter;
		odometryMsg.header.stamp = ros::Time::now();
		odometryMsg.header.frame_id = MAP_TF;
		odometryMsg.pose.pose = poseMsg.pose;
		odometryMsg.twist.twist = twistMsg.twist;
		rosOdometry.publish(odometryMsg);
		
		//wyślij ramkę (zakładamy ramkę map w 0,0,0)
		geometry_msgs::TransformStamped transMsg;
		transMsg.header.stamp = ros::Time::now();
		transMsg.header.frame_id = MAP_TF;
		transMsg.child_frame_id = "pseudovelma";
		transMsg.transform.translation.x = pose.pos.x;
		transMsg.transform.translation.y = pose.pos.y;
		transMsg.transform.translation.z = pose.pos.z;
		transMsg.transform.rotation.x = pose.rot.x;
		transMsg.transform.rotation.y = pose.rot.y;
		transMsg.transform.rotation.z = pose.rot.z;
		transMsg.transform.rotation.w = pose.rot.w;
		framePublisher.sendTransform(transMsg);

		counter++;
	}

	///Pobierz wiadomość od ROSa
	void OnRosMsg(const omnivelma_msgs::Vels::ConstPtr &msg)
	{
		if(!std::isnan(msg -> fl))
			flVel = msg -> fl;
		if(!std::isnan(msg -> fr))
			frVel = msg -> fr;
		if(!std::isnan(msg -> rl))
			rlVel = msg -> rl;
		if(!std::isnan(msg -> rr))
			rrVel = msg -> rr;
	}

	///Wskaźnik na model
	physics::ModelPtr model;

	///Wskaźnik na zdarzenie aktualizacji
	event::ConnectionPtr updateConnection;

	///Przedrostek modelu
	std::string linkPrefix;

	///Prędkości kół
	double rrVel;
	double rlVel;
	double frVel;
	double flVel;

	///Średnica kół
	double wheelRadius;

	///Szerokość modelu
	double modelWidth;

	///Długość modelu
	double modelLength;

	///Node dla ROSa
	std::unique_ptr<ros::NodeHandle> rosNode;

	///Odbiornik ROSa
	ros::Subscriber rosSub;

	///Nadajnik pozycji
	ros::Publisher rosPose;

	///Nadajnik prędkości
	ros::Publisher rosTwist;
	
	///Nadajnik odometrii
	ros::Publisher rosOdometry;

	///Licznik kroków symulacji
	unsigned int counter;
	
	//Nadajnik ramki
	tf2_ros::TransformBroadcaster framePublisher;

};

//zarejestruj wtyczkę
GZ_REGISTER_MODEL_PLUGIN(Pseudovelma)
}
