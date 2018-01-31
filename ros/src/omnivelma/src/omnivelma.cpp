#include <functional>
#include <string>
#include <iostream>
#include <cmath>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <ros/console.h>
#include <tf2_ros/transform_broadcaster.h>
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
#define MAP_TF "map"
#define MAX_FORCE 1000.0

namespace gazebo
{
///Klasa sterująca platformą
class Omnivelma : public ModelPlugin
{
public:
	Omnivelma()
	{
		counter = 0;
		velRR = 0;
		velFR = 0;
		velRL = 0;
		velFL = 0;
	}

public:
	///Uruchamiane na inicjalizację
	void Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
	{
		model = parent;

		updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&Omnivelma::OnUpdate, this));

		linkPrefix = std::string(model -> GetName()).append("::").append(MODEL_NAME).append("::");
		std::string topicPrefix = std::string("/").append(model -> GetName()).append("/");
		
		//odszukaj obiekty kół
		physics::LinkPtr wheelRR = model ->  GetLink(linkPrefix + "wheel_rr");
		physics::LinkPtr wheelRL = model ->  GetLink(linkPrefix + "wheel_rl");
		physics::LinkPtr wheelFR = model ->  GetLink(linkPrefix + "wheel_fr");
		physics::LinkPtr wheelFL = model ->  GetLink(linkPrefix + "wheel_fl");
		
		if(!wheelRR || !wheelRL || !wheelFR || !wheelFL)
		{
			ROS_FATAL_STREAM("Nie udało się znaleźć obiektów kół");
		}

		//odszukaj kolizje kół
		wheelRRCollision = wheelRR -> GetCollision("wheel_rr_collision");
		wheelRLCollision = wheelRL -> GetCollision("wheel_rl_collision");
		wheelFRCollision = wheelFR -> GetCollision("wheel_fr_collision");
		wheelFLCollision = wheelFL -> GetCollision("wheel_fl_collision");
		
		if(!wheelRRCollision || !wheelRLCollision || !wheelFRCollision || !wheelFLCollision)
		{
			ROS_FATAL_STREAM("Nie udało się znaleźć kolizji kół modelu");
		}
		
		//odszukaj przeguby
		motorRR = model -> GetJoint(linkPrefix + "motor_rr");
		motorRL = model -> GetJoint(linkPrefix + "motor_rl");
		motorFR = model -> GetJoint(linkPrefix + "motor_fr");
		motorFL = model -> GetJoint(linkPrefix + "motor_fl");
		//nadaj maksymalną siłę
		motorRR -> SetParam("fmax", 0, MAX_FORCE);
		motorRL -> SetParam("fmax", 0, MAX_FORCE);
		motorFR -> SetParam("fmax", 0, MAX_FORCE);
		motorFL -> SetParam("fmax", 0, MAX_FORCE);
		
		if(!motorRR || !motorRL || !motorFR || !motorFL)
		{
			ROS_FATAL_STREAM("Nie udało się znaleźć przegubów silników");
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
	///Ustaw prędkości kół z pól
	void SetVelocities()
	{
		//nadaj poprzednie prędkości kół
		if(!std::isnan(velFR))
			motorFR -> SetParam("vel", 0, velFR);
		if(!std::isnan(velFL))
			motorFL -> SetParam("vel", 0, velFL);
		if(!std::isnan(velRR))
			motorRR -> SetParam("vel", 0, velRR);
		if(!std::isnan(velRL))
			motorRL -> SetParam("vel", 0, velRL);
	}

	///Funkcja podłączana do zdarzenia aktualizacji
	void OnUpdate()
	{
		//ustaw kierunek wektorów tarcia
		//kierunek wektora o większym współczynniku tarcia jest ustalony w lokalnym, dla koła, układzie współrzędnych
		//zatem wektor należy obrócić zgodnie z modelem i odwrotnie do obrotu koła, aby wyjściowo był w płaszczyźnie platformy
		const math::Quaternion modelRot = model -> GetWorldPose().rot;
		const math::Quaternion wheelRRRot = wheelRRCollision -> GetWorldPose().rot;
		const math::Quaternion wheelRLRot = wheelRLCollision -> GetWorldPose().rot;
		const math::Quaternion wheelFRRot = wheelFRCollision -> GetWorldPose().rot;
		const math::Quaternion wheelFLRot = wheelFLCollision -> GetWorldPose().rot;
		
		wheelRRCollision -> GetSurface() -> FrictionPyramid() -> direction1 = wheelRRRot.RotateVectorReverse(modelRot.RotateVector(math::Vector3(AXIS_LENGTH, AXIS_LENGTH, 0)));
		wheelRLCollision -> GetSurface() -> FrictionPyramid() -> direction1 = wheelRLRot.RotateVectorReverse(modelRot.RotateVector(math::Vector3(-AXIS_LENGTH, AXIS_LENGTH, 0)));
		wheelFRCollision -> GetSurface() -> FrictionPyramid() -> direction1 = wheelFRRot.RotateVectorReverse(modelRot.RotateVector(math::Vector3(AXIS_LENGTH, -AXIS_LENGTH, 0)));
		wheelFLCollision -> GetSurface() -> FrictionPyramid() -> direction1 = wheelFLRot.RotateVectorReverse(modelRot.RotateVector(math::Vector3(-AXIS_LENGTH, -AXIS_LENGTH, 0)));

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
		twistMsg.header.frame_id = MAP_TF;
		rosTwist.publish(twistMsg);
		
		//wyślij ramkę (zakładamy ramkę map w 0,0,0)
		geometry_msgs::TransformStamped transMsg;
		transMsg.header.stamp = ros::Time::now();
		transMsg.header.frame_id = MAP_TF;
		transMsg.child_frame_id = "omnivelma";
		transMsg.transform.translation.x = pose.pos.x;
		transMsg.transform.translation.y = pose.pos.y;
		transMsg.transform.translation.z = pose.pos.z;
		transMsg.transform.rotation.x = pose.rot.x;
		transMsg.transform.rotation.y = pose.rot.y;
		transMsg.transform.rotation.z = pose.rot.z;
		transMsg.transform.rotation.w = pose.rot.w;
		framePublisher.sendTransform(transMsg);
		
		counter++;
		
		SetVelocities();
	}

	///Ustaw tarcia dla kół
	bool SetFriction(const omnivelma_msgs::SetFriction::Request& req, omnivelma_msgs::SetFriction::Response& res)
	{
		wheelRRCollision -> GetSurface() -> FrictionPyramid() -> SetMuPrimary(req.mu1);
		wheelRRCollision -> GetSurface() -> FrictionPyramid() -> SetMuSecondary(req.mu2);
		wheelRLCollision -> GetSurface() -> FrictionPyramid() -> SetMuPrimary(req.mu1);
		wheelRLCollision -> GetSurface() -> FrictionPyramid() -> SetMuSecondary(req.mu2);
		wheelFRCollision -> GetSurface() -> FrictionPyramid() -> SetMuPrimary(req.mu1);
		wheelFRCollision -> GetSurface() -> FrictionPyramid() -> SetMuSecondary(req.mu2);
		wheelFLCollision -> GetSurface() -> FrictionPyramid() -> SetMuPrimary(req.mu1);
		wheelFLCollision -> GetSurface() -> FrictionPyramid() -> SetMuSecondary(req.mu2);
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
		velRR = msg -> rr;
		velRL = msg -> rl;
		velFR = msg -> fr;
		velFL = msg -> fl;
	
		SetVelocities();
	}

	///Wskaźnik na model
	physics::ModelPtr model;

	///Wskaźnik na zdarzenie aktualizacji
	event::ConnectionPtr updateConnection;

	///Przedrostek modelu
	std::string linkPrefix;

	///Motory kół
	physics::JointPtr motorRR;
	physics::JointPtr motorRL;
	physics::JointPtr motorFR;
	physics::JointPtr motorFL;
	
	//Kolizje kół
	physics::CollisionPtr wheelRRCollision;
	physics::CollisionPtr wheelRLCollision;
	physics::CollisionPtr wheelFRCollision;
	physics::CollisionPtr wheelFLCollision;
	
	//prędkości kół
	double velRR;
	double velRL;
	double velFR;
	double velFL;

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
	
	//Nadajnik ramki
	tf2_ros::TransformBroadcaster framePublisher;

};

//zarejestruj wtyczkę
GZ_REGISTER_MODEL_PLUGIN(Omnivelma)
}
