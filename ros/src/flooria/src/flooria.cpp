#include <functional>
#include <string>
#include <iostream>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <flooria/SetFriction.h>

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
        model = parent;

        linkPrefix = MODEL_NAME.append("::").append(model -> GetName()).append("::");

        pyramid = model -> GetLink(linkPrefix + "base") -> GetCollision("collision") -> GetSurface() -> FrictionPyramid();

        //inicjalizacja ROSa
        if (!ros::isInitialized())
        {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, CLIENT_NAME, ros::init_options::NoSigintHandler);
			std::cout << "Initializacja ROSa we Floorii" << std::endl;
        }

        //stwórz Node dla ROSa
        rosNode.reset(new ros::NodeHandle());

        //stwórz serwer do ustawiania tarcia
		ros::AdvertiseServiceOptions aso = ros::AdvertiseServiceOptions::create<flooria::SetFriction>("/flooria/set_friction", std::bind(&Flooria::SetFriction, this, std::placeholders::_1, std::placeholders::_2), nullptr, nullptr);
        rosSrv = rosNode -> advertiseService(aso);

        std::cout << "Podłączono Floorię " << std::endl;
    }

private:
    bool SetFriction(flooria::SetFriction::Request& req, flooria::SetFriction::Response& res)
    {
        pyramid -> SetMuPrimary(req.mu1);
        pyramid -> SetMuSecondary(req.mu2);
        std::cout << "Ustawiono tarcia podłoża: " << req.mu1 << " " << req.mu2 << std::endl;
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

