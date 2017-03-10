#include <functional>
#include <string>
#include <iostream>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <omnivelma/Vels.h>
#include <omnivelma/SetFriction.h>
#include <omnivelma/Encoders.h>
#include <geometry_msgs/Pose.h>
#include <omnivelma/SetInertia.h>

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
    ///Uruchamiane na inicjalizację
    void Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
    {
        model = parent;

        updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&Omnivelma::OnUpdate, this));

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
            std::cout << "ROS initializowany w Omnivelmie" << std::endl;
        }

        //stwórz Node dla ROSa
        rosNode.reset(new ros::NodeHandle());

        //stwórz topic do odbierania prędkości
        rosSub = rosNode -> subscribe<omnivelma::Vels>("/omnivelma/vels", 1, std::bind(&Omnivelma::OnRosMsg, this, std::placeholders::_1));

        //stwórz topic do nadawania pozycji
        rosPub = rosNode -> advertise<geometry_msgs::Pose>("/omnivelma/pose", 1000);

        //stwórz topic do nadawania enkoderów
        rosEnc = rosNode -> advertise<omnivelma::Encoders>("/omnivelma/encoders", 1000);

        //stwórz serwer do odbierania tarcia
        ros::AdvertiseServiceOptions aso = ros::AdvertiseServiceOptions::create<omnivelma::SetFriction>("/omnivelma/set_friction", std::bind(&Omnivelma::SetFriction, this, std::placeholders::_1, std::placeholders::_2), nullptr, nullptr);
        rosSrv = rosNode -> advertiseService(aso);

        //stwórz serwer do odbierania inercji
        ros::AdvertiseServiceOptions asi = ros::AdvertiseServiceOptions::create<omnivelma::SetInertia>("/omnivelma/set_inertia", std::bind(&Omnivelma::SetInertia, this, std::placeholders::_1, std::placeholders::_2), nullptr, nullptr);
        rosIne = rosNode -> advertiseService(asi);

        std::cout << "Podłączono Omnivelmę " << std::endl;
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

        //wyślij enkodery
        omnivelma::Encoders encMsg;
        encMsg.vel_rr = motorRR -> GetVelocity(0);
        encMsg.angle_rr = motorRR -> GetAngle(0).Radian();
        encMsg.vel_rl = motorRL -> GetVelocity(0);
        encMsg.angle_rl = motorRL -> GetAngle(0).Radian();
        encMsg.vel_fr = motorFR -> GetVelocity(0);
        encMsg.angle_fr = motorFR -> GetAngle(0).Radian();
        encMsg.vel_fl = motorFL -> GetVelocity(0);
        encMsg.angle_fl = motorFL -> GetAngle(0).Radian();
        rosEnc.publish(encMsg);
    }
private:
    ///Ustaw tarcia dla kół
    bool SetFriction(const omnivelma::SetFriction::Request& req, omnivelma::SetFriction::Response& res)
    {
        pyramidRR -> SetMuPrimary(req.mu1);
        pyramidRR -> SetMuSecondary(req.mu2);
        pyramidRL -> SetMuPrimary(req.mu1);
        pyramidRL -> SetMuSecondary(req.mu2);
        pyramidFR -> SetMuPrimary(req.mu1);
        pyramidFR -> SetMuSecondary(req.mu2);
        pyramidFL -> SetMuPrimary(req.mu1);
        pyramidFL -> SetMuSecondary(req.mu2);
        std::cout << "Ustawiono tarcia: " << req.mu1 << " " << req.mu2 << std::endl;
        return true;
    }
    ///Ustaw masy, środki mas i tensor inercji.
    bool SetInertia(const omnivelma::SetInertia::Request& req, omnivelma::SetInertia::Response& res)
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
            std::cerr << "Nie udało się ustawić inercji: " << err.GetErrorStr() << std::endl;
            return false;
        }

        std::cout << "Ustawiono inercje: " << req.base.m << " " << req.front.m << " " << req.wheel.m << std::endl;
        return true;
    }

    ///Pobierz wiadomość od ROSa
private:
    void OnRosMsg(const omnivelma::Vels::ConstPtr& msg)
    {
        motorRR -> SetVelocity(0, msg -> rr);
        motorRL -> SetVelocity(0, msg -> rl);
        motorFR -> SetVelocity(0, msg -> fr);
        motorFL -> SetVelocity(0, msg -> fl);
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

    ///Nadajnik enkodera
    ros::Publisher rosEnc;

    ///Serwer ustawiania tarcia
    ros::ServiceServer rosSrv;

    ///Serwer ustawiania inercji
    ros::ServiceServer rosIne;

};

//zarejestruj wtyczkę
GZ_REGISTER_MODEL_PLUGIN(Omnivelma)
}
