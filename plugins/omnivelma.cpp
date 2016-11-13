#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

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
    }

public:
    ///Funkcja podłączana do zdarzenia aktualizacji
    void OnUpdate(const common::UpdateInfo& info)
    {

    }

private:
    ///Wskaźnik na model
    physics::ModelPtr model;

    ///Wskaźnik na zdarzenie aktualizacji
    event::ConnectionPtr updateConnection;
};

//zarejestruj wtyczkę
GZ_REGISTER_MODEL_PLUGIN(Omnivelma)
}
