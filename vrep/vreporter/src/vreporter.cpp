extern "C" {
#include <extApi.h>
}
#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <omnivelma_msgs/Vels.h>

#define PORT				19997
#define LUA_FUN_NAME		"setVrevelmaVels"

///Id połączenia z V-Repem
int clientID = -1;

///Funkcja wywoływana na odbiór wiadomości
void velsCallback(const omnivelma_msgs::Vels::ConstPtr& msg)
{
	float wheelValues[4] = {(float)(msg -> fr), (float)(msg -> fl), (float)(msg -> rl), (float)(msg -> rr)};
	
	//Połączenie, nazwa modelu, typ skryptu, funkcja, ilość int, wartości, ilość float, wartości, ilość stringów, wartości, długości buforów i bufory do zwracania wartości...
	int result = simxCallScriptFunction(clientID, "vrevelma", sim_scripttype_childscript, LUA_FUN_NAME, 0, nullptr, 4, wheelValues, 0, nullptr, 0, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, simx_opmode_blocking);
	if (result != simx_return_ok)
	{
		std::cerr << "Błąd wysyłania pakietu" << std::endl;
	}
}

int main(int argc, char** argv)
{
	//ros::init() modyfikuje argumenty
	ros::init(argc, argv, "vreporter");
	ros::NodeHandle handle;
	ros::Subscriber sub = handle.subscribe<omnivelma_msgs::Vels>("/vreporter/vels", 1, velsCallback);
	
	//adres, port, czy czekać na połączenie, czy łączyć w razie zerwania, timeout w ms, częstotliwość komunikacji
	clientID = simxStart((simxChar*) "127.0.0.1", PORT, true, true, 2000, 5);
	if(clientID != 0)
	{
		ROS_FATAL("Błąd łączenia z V-REPem");
	}
	ROS_DEBUG_STREM("Połączono z V-REPem na porcie " << PORT);

	ros::spin();
	simxFinish(clientID);
	return 0;
}
