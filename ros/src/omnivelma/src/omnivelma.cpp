#include <functional>
#include <map>
#include <string>
#include <iostream>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <thread>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <omnivelma/Vels.h>
#include <geometry_msgs/Pose.h>

#define MODEL_NAME std::string("omnivelma")
///Długość jest równa sqrt(2)/2 aby tworzyć kąt 45°
#define AXIS_LENGTH 0.707106781186548
#define CLIENT_NAME "gazebo_client"

namespace gazebo
{
///Klasa sterująca platformą
class Omnivelma : public ModelPlugin
{
public:
    ///Uruchamiane na inicjalizację
    void Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
    {
        this -> model = parent;

        this -> updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&Omnivelma::OnUpdate, this));

        //common::Logger logger("Omnivelma", common::Color::Purple.GetAsARGB(), common::Logger::LogType::STDOUT);
        //logger("Podłączono wtyczkę", 10);

        std::cout << "Podłączono wtyczkę do " << model -> GetName() << std::endl;
        linkPrefix = MODEL_NAME.append("::").append(model -> GetName()).append("::");

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
            char **argv = NULL;
            ros::init(argc, argv, CLIENT_NAME, ros::init_options::NoSigintHandler);
        }

        //stwórz Node dla ROSa
        this -> rosNode.reset(new ros::NodeHandle(CLIENT_NAME));

        //stwórz topic do odbierania wiadomości
        ros::SubscribeOptions so = ros::SubscribeOptions::create<omnivelma::Vels>("/" + model -> GetName() + "/vels", 1, std::bind(&Omnivelma::OnRosMsg, this, std::placeholders::_1), ros::VoidPtr(), &this -> rosQueue);
        this -> rosSub = this -> rosNode -> subscribe(so);

        //sruchom wątek odbierania
        this -> rosQueueThread = std::thread(std::bind(&Omnivelma::QueueThread, this));

        //stwórz topic do nadawania wiadomości
        this -> rosPub = this -> rosNode -> advertise<geometry_msgs::Pose>("/" + model -> GetName() + "/pose", 1000);
    }

public:
    ///Funkcja podłączana do zdarzenia aktualizacji
    void OnUpdate()
    {
        //ustaw kierunek wektorów tarcia
        math::Quaternion modelRot = model -> GetWorldPose().rot;
        pyramidRR -> direction1 = modelRot.RotateVector(math::Vector3(AXIS_LENGTH, AXIS_LENGTH, 0));
        pyramidRL -> direction1 = modelRot.RotateVector(math::Vector3(-AXIS_LENGTH, AXIS_LENGTH, 0));
        pyramidFR -> direction1 = modelRot.RotateVector(math::Vector3(AXIS_LENGTH, -AXIS_LENGTH, 0));
        pyramidFL -> direction1 = modelRot.RotateVector(math::Vector3(-AXIS_LENGTH, -AXIS_LENGTH, 0));

        //wyślij pozycję
        const math::Pose& pose = model -> GetWorldPose();
        geometry_msgs::Pose msg;
        msg.position.x = pose.pos.x;
        msg.position.y = pose.pos.y;
        msg.position.z = pose.pos.z;
        msg.orientation.x = pose.rot.x;
        msg.orientation.y = pose.rot.y;
        msg.orientation.z = pose.rot.z;
        msg.orientation.w = pose.rot.w;
        rosPub.publish(msg);
    }

    ///Pobierz wiadomość od ROSa
public:
    void OnRosMsg(const omnivelma::VelsConstPtr &msg)
    {
        std::cout << "Wiadomość: " << msg -> fl << " " << msg -> fr << " " << msg -> rl << " " << msg -> rr << std::endl;
        motorRR -> SetVelocity(0, msg -> rr);
        motorRL -> SetVelocity(0, msg -> rl);
        motorFR -> SetVelocity(0, msg -> fr);
        motorFL -> SetVelocity(0, msg -> fl);
    }

    ///Wątek odbioru wiadomości
private:
    void QueueThread()
    {
        static const double timeout = 0.01;
        while (this -> rosNode -> ok())
        {
            this -> rosQueue.callAvailable(ros::WallDuration(timeout));
        }
    }

private:
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
    ros::Publisher rosPub;

    ///Kolejka wiadomości
    ros::CallbackQueue rosQueue;

    ///Wątek kolejki
    std::thread rosQueueThread;

};

//zarejestruj wtyczkę
GZ_REGISTER_MODEL_PLUGIN(Omnivelma)
}
