#include <functional>
#include <string>
#include <iostream>
#include <cmath>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <ros/console.h>
#include <omnivelma_msgs/Vels.h>
#include <omnivelma_msgs/SetFriction.h>
#include <omnivelma_msgs/EncodersStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <omnivelma_msgs/SetInertia.h>

#define MODEL_NAME std::string("omnivelma")
///Długość jest równa sqrt(2)/2 aby tworzyć kąt 45°
#define AXIS_LENGTH 0.707106781186548
#define CLIENT_NAME "gazebo_ros"

namespace gazebo
{
///Klasa sterująca platformą
class Omnivelma : public ModelPlugin
{
public:
	Omnivelma()
	{
		counter = 0;
	}

public:
	///Uruchamiane na inicjalizację
	void Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
	{
		model = parent;

		updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&Omnivelma::OnUpdate, this));

		linkPrefix = std::string(model -> GetName()).append("::").append(MODEL_NAME).append("::");
		std::string topicPrefix = std::string("/").append(model -> GetName()).append("/");

		pyramidRR = model -> GetLink(linkPrefix + "wheel_rr") -> GetCollision("wheel_rr_collision") -> GetSurface() -> FrictionPyramid();
		pyramidRL = model -> GetLink(linkPrefix + "wheel_rl") -> GetCollision("wheel_rl_collision") -> GetSurface() -> FrictionPyramid();
		pyramidFR = model -> GetLink(linkPrefix + "wheel_fr") -> GetCollision("wheel_fr_collision") -> GetSurface() -> FrictionPyramid();
		pyramidFL = model -> GetLink(linkPrefix + "wheel_fl") -> GetCollision("wheel_fl_collision") -> GetSurface() -> FrictionPyramid();

		motorRR = model -> GetJoint(linkPrefix + "motor_rr");
		motorRL = model -> GetJoint(linkPrefix + "motor_rl");
		motorFR = model -> GetJoint(linkPrefix + "motor_fr");
		motorFL = model -> GetJoint(linkPrefix + "motor_fl");

		//inicjalizacja ROSa
		if (!ros::isInitialized())
		{
			int argc = 0;
			char **argv = nullptr;
			ros::init(argc, argv, CLIENT_NAME, ros::init_options::NoSigintHandler);
		}

		//stwórz Node dla ROSa
		rosNode.reset(new ros::NodeHandle());

		//stwórz topic do odbierania prędkości kół
		rosSub = rosNode -> subscribe<omnivelma_msgs::Vels>(topicPrefix + "vels", 1, std::bind(&Omnivelma::OnRosMsg, this, std::placeholders::_1));
		if(!rosSub)
		{
			ROS_FATAL_STREAM("Nie udało się stworzyć odbiornika " << topicPrefix << "vels");
		}

		//stwórz topic do nadawania pozycji
		rosPose = rosNode -> advertise<geometry_msgs::PoseStamped>(topicPrefix + "pose", 1000);
		if(!rosPose)
		{
			ROS_FATAL_STREAM("Nie udało się stworzyć nadajnika " << topicPrefix << "pose");
		}

		//stwórz topic do nadawania enkoderów
		rosEnc = rosNode -> advertise<omnivelma_msgs::EncodersStamped>(topicPrefix + "encoders", 1000);
		if(!rosEnc)
		{
			ROS_FATAL_STREAM("Nie udało się stworzyć nadajnika " << topicPrefix << "encoders");
		}

		//stwórz topic do nadawania prędkości
		rosTwist = rosNode -> advertise<geometry_msgs::TwistStamped>(topicPrefix + "twist", 1000);
		if(!rosTwist)
		{
			ROS_FATAL_STREAM("Nie udało się stworzyć nadajnika " << topicPrefix << "twist");
		}

		//stwórz serwer do odbierania tarcia
		ros::AdvertiseServiceOptions aso = ros::AdvertiseServiceOptions::create<omnivelma_msgs::SetFriction>(topicPrefix + "set_friction", std::bind(&Omnivelma::SetFriction, this, std::placeholders::_1, std::placeholders::_2), nullptr, nullptr);
		rosFri = rosNode -> advertiseService(aso);
		if(!rosFri)
		{
			ROS_FATAL_STREAM("Nie udało się stworzyć serwera " << topicPrefix << "set_friction");
		}

		//stwórz serwer do odbierania inercji
		ros::AdvertiseServiceOptions asi = ros::AdvertiseServiceOptions::create<omnivelma_msgs::SetInertia>(topicPrefix + "set_inertia", std::bind(&Omnivelma::SetInertia, this, std::placeholders::_1, std::placeholders::_2), nullptr, nullptr);
		rosIne = rosNode -> advertiseService(asi);
		if(!rosIne)
		{
			ROS_FATAL_STREAM("Nie udało się stworzyć serwera " << topicPrefix << "set_inertia");
		}
		
		//powiadom o gotowości
		ROS_DEBUG_STREAM("Omnivelma zainicjalizowana");
	}


private:
	///Funkcja podłączana do zdarzenia aktualizacji
	void OnUpdate()
	{
		//ustaw kierunek wektorów tarcia
		const math::Quaternion modelRot = model -> GetWorldPose().rot;
		pyramidRR -> direction1 = modelRot.RotateVector(math::Vector3(AXIS_LENGTH, AXIS_LENGTH, 0));
		pyramidRL -> direction1 = modelRot.RotateVector(math::Vector3(-AXIS_LENGTH, AXIS_LENGTH, 0));
		pyramidFR -> direction1 = modelRot.RotateVector(math::Vector3(AXIS_LENGTH, -AXIS_LENGTH, 0));
		pyramidFL -> direction1 = modelRot.RotateVector(math::Vector3(-AXIS_LENGTH, -AXIS_LENGTH, 0));

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
		poseMsg.header.frame_id = "1";
		rosPose.publish(poseMsg);

		//wyślij enkodery
		omnivelma_msgs::EncodersStamped encMsg;
		encMsg.encoders.velocity.rr = motorRR -> GetVelocity(0);
		encMsg.encoders.angle.rr = motorRR -> GetAngle(0).Radian();
		encMsg.encoders.velocity.rl = motorRL -> GetVelocity(0);
		encMsg.encoders.angle.rl = motorRL -> GetAngle(0).Radian();
		encMsg.encoders.velocity.fr = motorFR -> GetVelocity(0);
		encMsg.encoders.angle.fr = motorFR -> GetAngle(0).Radian();
		encMsg.encoders.velocity.fl = motorFL -> GetVelocity(0);
		encMsg.encoders.angle.fl = motorFL -> GetAngle(0).Radian();
		encMsg.header.seq = counter;
		encMsg.header.stamp = ros::Time::now();
		encMsg.header.frame_id = "0";
		rosEnc.publish(encMsg);

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
		twistMsg.header.frame_id = "1";
		
		rosTwist.publish(twistMsg);
		counter++;
	}

	///Ustaw tarcia dla kół
	bool SetFriction(const omnivelma_msgs::SetFriction::Request& req, omnivelma_msgs::SetFriction::Response& res)
	{
		pyramidRR -> SetMuPrimary(req.mu1);
		pyramidRR -> SetMuSecondary(req.mu2);
		pyramidRL -> SetMuPrimary(req.mu1);
		pyramidRL -> SetMuSecondary(req.mu2);
		pyramidFR -> SetMuPrimary(req.mu1);
		pyramidFR -> SetMuSecondary(req.mu2);
		pyramidFL -> SetMuPrimary(req.mu1);
		pyramidFL -> SetMuSecondary(req.mu2);
		ROS_DEBUG("Ustawiono tarcia: %lf %lf", req.mu1, req.mu2);
		return true;
	}
	///Ustaw masy, środki mas i tensor inercji.
	bool SetInertia(const omnivelma_msgs::SetInertia::Request& req, omnivelma_msgs::SetInertia::Response& res)
	{
		physics::LinkPtr basePtr = model -> GetLink(linkPrefix + "base");
		physics::LinkPtr frontPtr = model -> GetLink(linkPrefix + "front");
		physics::LinkPtr wheelRRPtr = model -> GetLink(linkPrefix + "wheel_rr");
		physics::LinkPtr wheelRLPtr = model -> GetLink(linkPrefix + "wheel_rl");
		physics::LinkPtr wheelFRPtr = model -> GetLink(linkPrefix + "wheel_fr");
		physics::LinkPtr wheelFLPtr = model -> GetLink(linkPrefix + "wheel_fl");

		physics::InertialPtr baseIne = basePtr -> GetInertial();
		physics::InertialPtr frontIne = frontPtr -> GetInertial();
		physics::InertialPtr wheelRRIne = wheelRRPtr -> GetInertial();
		physics::InertialPtr wheelRLIne = wheelRLPtr -> GetInertial();
		physics::InertialPtr wheelFRIne = wheelFRPtr -> GetInertial();
		physics::InertialPtr wheelFLIne = wheelFLPtr -> GetInertial();

		baseIne -> SetMass(req.base.m);
		baseIne -> SetInertiaMatrix(req.base.ixx, req.base.iyy, req.base.izz, req.base.ixy, req.base.ixz, req.base.iyz);
		baseIne -> SetCoG(req.base.com.x, req.base.com.y, req.base.com.z);

		frontIne -> SetMass(req.front.m);
		frontIne -> SetInertiaMatrix(req.front.ixx, req.front.iyy, req.front.izz, req.front.ixy, req.front.ixz, req.front.iyz);
		frontIne -> SetCoG(req.front.com.x, req.front.com.y, req.front.com.z);

		wheelRRIne -> SetMass(req.wheel.m);
		wheelRRIne -> SetInertiaMatrix(req.wheel.ixx, req.wheel.iyy, req.wheel.izz, req.wheel.ixy, req.wheel.ixz, req.wheel.iyz);
		wheelRRIne -> SetCoG(req.wheel.com.x, req.wheel.com.y, req.wheel.com.z);
		wheelRLIne -> SetMass(req.wheel.m);
		wheelRLIne -> SetInertiaMatrix(req.wheel.ixx, req.wheel.iyy, req.wheel.izz, req.wheel.ixy, req.wheel.ixz, req.wheel.iyz);
		wheelRLIne -> SetCoG(req.wheel.com.x, req.wheel.com.y, req.wheel.com.z);
		wheelFRIne -> SetMass(req.wheel.m);
		wheelFRIne -> SetInertiaMatrix(req.wheel.ixx, req.wheel.iyy, req.wheel.izz, req.wheel.ixy, req.wheel.ixz, req.wheel.iyz);
		wheelFRIne -> SetCoG(req.wheel.com.x, req.wheel.com.y, req.wheel.com.z);
		wheelFLIne -> SetMass(req.wheel.m);
		wheelFLIne -> SetInertiaMatrix(req.wheel.ixx, req.wheel.iyy, req.wheel.izz, req.wheel.ixy, req.wheel.ixz, req.wheel.iyz);
		wheelFLIne -> SetCoG(req.wheel.com.x, req.wheel.com.y, req.wheel.com.z);

		//Liczenie masy z nowych macierzy może wywołać błędy, np. gdy masa jest zerowa
		try
		{
			frontPtr -> UpdateMass();
			basePtr -> UpdateMass();
			wheelRRPtr -> UpdateMass();
			wheelRLPtr -> UpdateMass();
			wheelFRPtr -> UpdateMass();
			wheelFLPtr -> UpdateMass();
		}
		catch(common::Exception err)
		{
			ROS_ERROR_STREAM("Nie udało się ustawić inercji: " << err.GetErrorStr());
			return false;
		}

		ROS_DEBUG("Ustawiono inercje");
		return true;
	}

	///Pobierz wiadomość od ROSa
	void OnRosMsg(const omnivelma_msgs::Vels::ConstPtr& msg)
	{
		if(!std::isnan(msg -> rr))
			motorRR -> SetVelocity(0, msg -> rr);
		if(!std::isnan(msg -> rl))
			motorRL -> SetVelocity(0, msg -> rl);
		if(!std::isnan(msg -> fr))
			motorFR -> SetVelocity(0, msg -> fr);
		if(!std::isnan(msg -> fl))
			motorFL -> SetVelocity(0, msg -> fl);
	}

	///Wskaźnik na model
	physics::ModelPtr model;

	///Wskaźnik na zdarzenie aktualizacji
	event::ConnectionPtr updateConnection;

	///Przedrostek modelu
	std::string linkPrefix;

	///Piramidy tarcia
	physics::FrictionPyramidPtr pyramidRR;
	physics::FrictionPyramidPtr pyramidRL;
	physics::FrictionPyramidPtr pyramidFR;
	physics::FrictionPyramidPtr pyramidFL;

	///Motory kół
	physics::JointPtr motorRR;
	physics::JointPtr motorRL;
	physics::JointPtr motorFR;
	physics::JointPtr motorFL;

	///Node dla ROSa
	std::unique_ptr<ros::NodeHandle> rosNode;

	///Odbiornik prędkości kół
	ros::Subscriber rosSub;

	///Nadajnik pozycji
	ros::Publisher rosPose;

	///Nadajnik enkodera
	ros::Publisher rosEnc;

	///Nadajnik prędkości
	ros::Publisher rosTwist;

	///Serwer ustawiania tarcia
	ros::ServiceServer rosFri;

	///Serwer ustawiania inercji
	ros::ServiceServer rosIne;

	///Licznik kroków
	unsigned int counter;

};

//zarejestruj wtyczkę
GZ_REGISTER_MODEL_PLUGIN(Omnivelma)
}
