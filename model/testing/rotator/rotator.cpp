 #ifndef _ROTATOR_HH_
 #define _ROTATOR_HH_
 
 #include <boost/bind.hpp>
 #include <gazebo/gazebo.hh>
 #include <gazebo/physics/physics.hh>
 #include <gazebo/common/common.hh>
 #include <gazebo/transport/transport.hh>
 #include <gazebo/msgs/msgs.hh>
#include <string>
#include <iostream>
#include <thread>
 #include <stdio.h>
 #include "ros/ros.h"
 #include "ros/callback_queue.h"
 #include "ros/subscribe_options.h"
 #include "std_msgs/Float32.h"
 
 namespace gazebo
 {
	 /// \brief A plugin to control a Velodyne sensor.
	 class Rotator : public ModelPlugin
	 {
		 /// \brief A node use for ROS transport
		 private: std::unique_ptr<ros::NodeHandle> rosNode;
		 
		 /// \brief A ROS subscriber
		 private: ros::Subscriber rosSub;
		 
		 /// \brief A ROS callbackqueue that helps process messages
		 private: ros::CallbackQueue rosQueue;
		 
		 /// \brief A thread the keeps running the rosQueue
		 private: std::thread rosQueueThread;
		public: Rotator() 
{
			velocity = 0;
		}
		 
		private: double velocity;
		
		/// \brief A node used for transport
		private: transport::NodePtr node;
		
		/// \brief A subscriber to a named topic.
		private: transport::SubscriberPtr sub;
		
		/// \brief Handle an incoming message from ROS
		/// \param[in] _msg A float value that is used to set the velocity
		/// of the Velodyne.
		public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
		{
			this->SetVelocity(_msg->data);
		}
		
		/// \brief ROS helper function that processes messages
		private: void QueueThread()
		{
			static const double timeout = 0.01;
			while (this->rosNode->ok())
			{
				this->rosQueue.callAvailable(ros::WallDuration(timeout));
			}
		}
		 
		 public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
		 {
			 // Store the pointer to the model
			 this->model = _parent;
			 
			 std::cerr << this -> model -> GetName() << std::endl;
			 
			 // Check that the velocity element exists, then read the value
			 if (_sdf->HasElement("velocity"))
				 velocity = _sdf->Get<double>("velocity");
			 
			 
			 // Listen to the update event. This event is broadcast every
			 // simulation iteration.
			 this->updateConnection = event::Events::ConnectWorldUpdateBegin(
				 boost::bind(&Rotator::OnUpdate, this, _1));
			 
			 // Create the node
			 this->node = transport::NodePtr(new transport::Node());
			 this->node->Init(this->model->GetWorld()->GetName());
			 
			 // Create a topic name
			 std::string topicName = this->model->GetName() + "/rotator_cmd";
			 
			 // Subscribe to the topic, and register a callback
			 this->sub = this->node->Subscribe(topicName, &Rotator::OnMsg, this);
			 
			 if (!ros::isInitialized())
			 {
				 int argc = 0;
				 char **argv = NULL;
				 ros::init(argc, argv, "gazebo_client",
						   ros::init_options::NoSigintHandler);
			 }
			 
			 // Create our ROS node. This acts in a similar manner to
			 // the Gazebo node
			 this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
			 
			 // Create a named topic, and subscribe to it.
			 ros::SubscribeOptions so =
			 ros::SubscribeOptions::create<std_msgs::Float32>(
				 "/" + this->model->GetName() + "/rotator_cmd",
															  1,
													 boost::bind(&Rotator::OnRosMsg, this, _1),
															  ros::VoidPtr(), &this->rosQueue);
			 this->rosSub = this->rosNode->subscribe(so);
			 
			 // Spin up the queue helper thread.
			 this->rosQueueThread =
			 std::thread(std::bind(&Rotator::QueueThread, this));
			 
		 }
		 
		 // Called by the world update start event
		 public: void OnUpdate(const common::UpdateInfo & /*_info*/)
		 {
			 // Apply a small linear velocity to the model.
			 this->model->SetLinearVel(math::Vector3(velocity, 0, 0));
		 }
		 
		 private: void OnMsg(ConstVector3dPtr &_msg)
		 {
			 this->SetVelocity(_msg->x());
		 }
		 
		 // Pointer to the model
		 private: physics::ModelPtr model;
		 
		 // Pointer to the update event connection
		 private: event::ConnectionPtr updateConnection;
		 
		 public: void SetVelocity(const double &_vel)
		 {
			 this -> velocity = _vel;
		 }
		 
		 
		 
		 
	 };
	 
	 // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
	 GZ_REGISTER_MODEL_PLUGIN(Rotator)
 }
 #endif
