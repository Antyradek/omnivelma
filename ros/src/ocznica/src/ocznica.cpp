#include <functional>
#include <string>
#include <iostream>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/math/gzmath.hh>
#include <ros/ros.h>
#include <ocznica/Relative.h>
#include <cmath>

namespace gazebo
{
///Klasa sterująca platformą
class Ocznica : public WorldPlugin
{
public:

    ///Uruchamiane na inicjalizację
    void Load(physics::WorldPtr world, sdf::ElementPtr sdfElement)
    {

        //podłączenie do wydarznia aktualizacji
        this -> updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&Ocznica::OnUpdate, this));

        std::cout << "Ocznica obserwuje" << std::endl;

        //inicjalizacja ROSa
        if (!ros::isInitialized())
        {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
        }

        //stwórz Node dla ROSa
        this -> rosNode.reset(new ros::NodeHandle("gazebo_client"));

        //stwórz topic do nadawania wiadomości
        this -> rosPub = this -> rosNode -> advertise<ocznica::Relative>("ocznica/relative", 1000);

        //znajdź modele
        this -> omnivelma = world -> GetModel("omnivelma");
        this -> pseudovelma = world -> GetModel("pseudovelma");
    }

public:
    ///Funkcja podłączana do zdarzenia aktualizacji
    void OnUpdate()
    {
        const math::Pose& omniPose = omnivelma -> GetWorldPose();
        const math::Pose& pseudoPose = pseudovelma -> GetWorldPose();
        double dist = omniPose.pos.Distance(pseudoPose.pos);
        //to działa jedynie dla małych kątów!
        double angle = sqrt(pow(omniPose.rot.x - pseudoPose.rot.x,2) + pow(omniPose.rot.y - pseudoPose.rot.y,2) + pow(omniPose.rot.z - pseudoPose.rot.z,2) + pow(omniPose.rot.w - pseudoPose.rot.w,2));

        ocznica::Relative msg;
        msg.distance = dist;
        msg.angle = angle;
        rosPub.publish(msg);
    }

private:

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

};

//zarejestruj wtyczkę
GZ_REGISTER_WORLD_PLUGIN(Ocznica)
}
