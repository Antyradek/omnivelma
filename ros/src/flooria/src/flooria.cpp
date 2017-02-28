#include <functional>
#include <string>
#include <iostream>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <thread>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <flooria/Friction.h>

#define MODEL_NAME std::string("flooria")
#define CLIENT_NAME "gazebo_ros"

namespace gazebo
{
///Klasa sterująca platformą
class Flooria : public ModelPlugin
{
public:
    ///Uruchamiane na inicjalizację
    void Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
    {
        this -> model = parent;

        linkPrefix = MODEL_NAME.append("::").append(model -> GetName()).append("::");

        pyramid = model -> GetLink(linkPrefix + "base") -> GetCollision("collision") -> GetSurface() -> FrictionPyramid();

        //inicjalizacja ROSa
        if (!ros::isInitialized())
        {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, CLIENT_NAME, ros::init_options::NoSigintHandler);
        }

        //stwórz Node dla ROSa
        this -> rosNode.reset(new ros::NodeHandle(CLIENT_NAME));

        //stwórz kolejkę wiadomości
        this -> rosQueueThread = std::thread(std::bind(&Flooria::QueueThread, this));

        //stwórz serwer do ustawiania tarcia
        ros::AdvertiseServiceOptions aso = ros::AdvertiseServiceOptions::create<flooria::Friction>("/flooria/set_friction", std::bind(&Flooria::SetFriction, this, std::placeholders::_1, std::placeholders::_2), ros::VoidPtr(), &this -> rosQueue);
        this -> rosSrv = this -> rosNode -> advertiseService(aso);

        std::cout << "Podłączono Floorię " << std::endl;
    }

private:
    bool SetFriction(flooria::Friction::Request  &req, flooria::Friction::Response &res)
    {
        pyramid -> SetMuPrimary(req.mu1);
        pyramid -> SetMuSecondary(req.mu2);
        std::cout << "Ustawiono tarcia podłoża: " << req.mu1 << " " << req.mu2 << std::endl;
        return true;
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

    ///Wskaźnik piramidy tarcia
    physics::FrictionPyramidPtr pyramid;

    ///Przedrostek modelu
    std::string linkPrefix;

    ///Node dla ROSa
    std::unique_ptr<ros::NodeHandle> rosNode;

    ///Serwer ustawiania tarcia
    ros::ServiceServer rosSrv;

    ///Kolejka wiadomości
    ros::CallbackQueue rosQueue;

    ///Wątek kolejki
    std::thread rosQueueThread;

};

//zarejestruj wtyczkę
GZ_REGISTER_MODEL_PLUGIN(Flooria)
}

