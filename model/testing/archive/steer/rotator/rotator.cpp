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
 #include <stdio.h>
 
 namespace gazebo
 {
	 /// \brief A plugin to control a Velodyne sensor.
	 class Rotator : public ModelPlugin
	 {
		public: Rotator() {
			velocity = 0;
		}
// 		 
// 		 /// \brief The load function is called by Gazebo when the plugin is
// 		 /// inserted into simulation
// 		 /// \param[in] _model A pointer to the model that this plugin is
// 		 /// attached to.
// 		 /// \param[in] _sdf A pointer to the plugin's SDF element.
// 		 public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
// 		 {
// 			
// 			 
// 			 // Store the model pointer for convenience.
// 			 this->model = _model;
// 			 
// 			 // Get the first joint. We are making an assumption about the model
// 			 // having one joint that is the rotational joint.
// 			 this->joint = this->model->GetJoints()[0];
// 			 
// 			 // Setup a P-controller, with a gain of 0.1.
// 			 this->pid = common::PID(0.1, 0, 0);
// 			 
// 			 // Apply the P-controller to the joint.
// 			 this->model->GetJointController()->SetVelocityPID(
// 				 this->joint->GetScopedName(), this->pid);
// 			 
// 			 // Set the joint's target velocity. This target velocity is just
// 			 // for demonstration purposes.
// 			 this->model->GetJointController()->SetVelocityTarget(
// 				 this->joint->GetScopedName(), 10.0);
//		 }
// 	/*/*/*/*	 
// 		 /// \brief A node used for transport
// 		 private: transport::NodePtr node;
// 		 
// 		 /// \brief A subscriber to a named topic.
// 		 private: transport::SubscriberPtr sub;
// 		 
// 		 /// \brief Pointer to the model.
// 		 private: physics::ModelPtr model;
// 		 
// 		 /// \brief Pointer to the joint.
// 		 private: physics::JointPtr joint;
// 		 
// 		 /// \brief A PID controller for the joint.
// 		 private: common::PID pid;*/*/*/*/
		 
		private: double velocity;
		
		/// \brief A node used for transport
		private: transport::NodePtr node;
		
		/// \brief A subscriber to a named topic.
		private: transport::SubscriberPtr sub;
		 
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
