#include <functional>
#include <string>
#include <iostream>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/math/gzmath.hh>
#include <thread>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <pseudovelma/Vels.h>
#include <geometry_msgs/Pose.h>

#define MODEL_NAME std::string("pseudovelma")

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
		flVel = 0; 	frVel = 0;
		rlVel = 0; 	rrVel = 0;

        //z modelu
        wheelRadius = 0.1;
        modelWidth = 0.76;
        modelLength = 0.72;
    }

    ///Uruchamiane na inicjalizację
    void Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
    {
        this -> model = parent;

        //podłączenie do wydarznia aktualizacji
        this -> updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&Pseudovelma::OnUpdate, this));

        linkPrefix = MODEL_NAME.append("::").append(model -> GetName()).append("::");

        //inicjalizacja ROSa
        if (!ros::isInitialized())
        {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "gazebo_ros", ros::init_options::NoSigintHandler);
        }

        //stwórz Node dla ROSa
        this -> rosNode.reset(new ros::NodeHandle("pseudovelma"));

        //Stwórz topic do odbierania wiadomości
        ros::SubscribeOptions so = ros::SubscribeOptions::create<pseudovelma::Vels>("/pseudovelma/vels", 1, std::bind(&Pseudovelma::OnRosMsg, this, std::placeholders::_1), ros::VoidPtr(), &this -> rosQueue);
        this -> rosSub = this -> rosNode -> subscribe(so);

        //Uruchom wątek odbierania
        this -> rosQueueThread = std::thread(std::bind(&Pseudovelma::QueueThread, this));

        //stwórz topic do nadawania wiadomości
        this -> rosPub = this -> rosNode -> advertise<geometry_msgs::Pose>("/pseudovelma/pose", 1000);

		std::cout << "Podłączono Pseudovelmę " << std::endl;
    }

public:
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
    void OnRosMsg(const pseudovelma::Vels::ConstPtr &msg)
    {
        flVel = msg -> fl;
        frVel = msg -> fr;
        rlVel = msg -> rl;
        rrVel = msg -> rr;
		std::cout << "Wiadomość: " << msg -> fl << " " << msg -> fr << " " << msg -> rl << " " << msg -> rr << std::endl;
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

    ///Prękości kół
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
    ros::Publisher rosPub;

    ///Kolejka wiadomości
    ros::CallbackQueue rosQueue;

    ///Wątek kolejki
    std::thread rosQueueThread;

};

//zarejestruj wtyczkę
GZ_REGISTER_MODEL_PLUGIN(Pseudovelma)
}
