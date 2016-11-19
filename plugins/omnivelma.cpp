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
    // Get the wheel's axis in the world frame.
    math::Pose const& wheel_world_pose = model->GetLink(wheelLinkName.c_str())->GetWorldPose();
    math::Matrix3 wheel_world_mat3 = wheel_world_pose.rot.GetAsMatrix3();
    math::Vector3 wheel_world_y(wheel_world_mat3[0][1], wheel_world_mat3[1][1], wheel_world_mat3[2][1]);

    // Compute the new roll axis (rotate the wheel axis 45° around the world Z).
    math::Quaternion rot(math::Vector3(0.0, 0.0, 1.0), rotationSide * math::Angle::Pi.Radian() / 4.0);
    math::Vector3 newAxis_world = rot.RotateVector(wheel_world_y);
    math::Vector3 newAxis_local = wheel_world_pose.rot.RotateVectorReverse(newAxis_world);

    // Set the new axis.
    model->GetJoint(rollerJointName.c_str())->SetAxis(0, newAxis_local);


  }

private:
  ///Funkcja podłączana do zdarzenia aktualizacji
  void OnUpdate(const common::UpdateInfo& info)
  {
    setRollAxis(linkPrefix + "virtual_wheel_fl",  linkPrefix + "roller_fl",   1.0);
    setRollAxis(linkPrefix + "virtual_wheel_rl",   linkPrefix + "roller_rl",   -1.0);
    setRollAxis(linkPrefix + "virtual_wheel_rr",  linkPrefix + "roller_rr",   1.0);
    setRollAxis(linkPrefix + "virtual_wheel_fr", linkPrefix + "roller_fr", -1.0);
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
