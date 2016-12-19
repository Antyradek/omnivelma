#include <functional>
#include <map>
#include <string>
#include <iostream>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/math/gzmath.hh>

//#include "ros/ros.h"
//#include "ros/callback_queue.h"
//#include "ros/subscribe_options.h"
//#include "std_msgs/Float32.h"

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

		//common::Logger logger("Omnivelma", common::Color::Purple.GetAsARGB(), common::Logger::LogType::STDOUT);
		//logger("Podłączono wtyczkę", 10);

		std::cout << "Podłączono wtyczkę do " << model -> GetName() << std::endl;
		linkPrefix = MODEL_NAME.append("::").append(model -> GetName()).append("::");


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

};

//zarejestruj wtyczkę
GZ_REGISTER_MODEL_PLUGIN(Pseudovelma)
}
