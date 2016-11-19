#include <functional>
#include <map>
#include <string>
#include <iostream>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#define MODEL_NAME std::string("omnivelma")
///Długość jest równa sqrt(2)/2 aby tworzyć kąt 45°
#define AXIS_LENGTH 0.707

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

        //podłączenie do wydarznia aktualizacji
        this -> updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&Omnivelma::OnUpdate, this, std::placeholders::_1));

        //common::Logger logger("Omnivelma", common::Color::Purple.GetAsARGB(), common::Logger::LogType::STDOUT);
        //logger("Podłączono wtyczkę", 10);

        std::cout << "Podłączono wtyczkę do " << model -> GetName() << std::endl;
        linkPrefix = MODEL_NAME.append("::").append(model -> GetName()).append("::");

        jointController = model -> GetJointController();
        joints = jointController -> GetJoints();

        std::cout << "Przeguby:" << std::endl;
        for(auto& x : joints)
        {
            std:: cout << x.first << std::endl;
        }

    }

private:
    void setRollAxis(std::string const& wheelLinkName, std::string const& rollerJointName, double rotationSide)
    {
        physics::LinkPtr baseLink = model -> GetLink(linkPrefix + "base");

        math::Vector3 baseUp = baseLink -> GetWorldPose().rot.GetZAxis();
        math::Quaternion globalRollerRot = math::Quaternion(baseUp, rotationSide);
        math::Vector3 wheelRight = model -> GetLink(wheelLinkName) -> GetWorldPose().rot.GetXAxis();
        double wheelRoll = model -> GetLink(wheelLinkName) -> GetRelativePose().rot.GetRoll();
        math::Quaternion invWheelRot = math::Quaternion(wheelRight, -wheelRoll);

        math::Vector3 rollerVect = baseLink -> GetWorldPose().rot.GetYAxis();
        rollerVect = globalRollerRot.RotateVector(rollerVect);
        rollerVect = invWheelRot.RotateVector(rollerVect);
        model -> GetJoint(rollerJointName) -> SetAxis(0, rollerVect);
    }

private:
    ///Funkcja podłączana do zdarzenia aktualizacji
    void OnUpdate(const common::UpdateInfo& info)
    {
        setRollAxis(linkPrefix + "virtual_wheel_fl", linkPrefix + "roller_fl", math::Angle::Pi.Radian() * 0.75);
        setRollAxis(linkPrefix + "virtual_wheel_rl", linkPrefix + "roller_rl", math::Angle::Pi.Radian() * 0.25);
        setRollAxis(linkPrefix + "virtual_wheel_rr", linkPrefix + "roller_rr", -math::Angle::Pi.Radian() * 0.25);
        setRollAxis(linkPrefix + "virtual_wheel_fr", linkPrefix + "roller_fr", -math::Angle::Pi.Radian() * 0.75);
    }

private:
    ///Wskaźnik na model
    physics::ModelPtr model;

    ///Wskaźnik na zdarzenie aktualizacji
    event::ConnectionPtr updateConnection;

    ///Przedrostek modelu
    std::string linkPrefix;

    ///Mapa kół do wspaźników na rolki
    //std::map<std::string, physics::JointPtr> rollers;

    ///Mapa przegubów
    std::map<std::string, physics::JointPtr> joints;

    ///Kontroler
    physics::JointControllerPtr jointController;
};

//zarejestruj wtyczkę
GZ_REGISTER_MODEL_PLUGIN(Omnivelma)
}
