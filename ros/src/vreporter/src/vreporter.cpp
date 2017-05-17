extern "C" {
#include <extApi.h>
}
#include <iostream>

#define PORT				19997
#define LUA_FUN_NAME		"setOmnivelmaVels"

int main()
{
	int clientID = simxStart((simxChar*) "127.0.0.1", PORT, true, true, 2000, 5);
	if(clientID == -1)
	{
		std::cerr << "Błąd łączenia z V-REPem" << std::endl;
		return -1;
	}
	std::cout << "Połączono z V-REPem na porcie " << PORT << std::endl;
	float wheelValues[4] = {1.0f, -1.0f, 1.0f, -1.0f};
	//Połączenie, nazwa modelu, typ skryptu, funkcja, ilość int, wartości, ilość float, wartości, ilość stringów, wartości, długości buforów i bufory do zwracania wartości...
	int result = simxCallScriptFunction(clientID, "omnivelma", sim_scripttype_childscript, LUA_FUN_NAME, 0, nullptr, 4, wheelValues, 0, nullptr, 0, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, simx_opmode_blocking);
	if (result != simx_return_ok)
	{
		std::cerr << "Błąd wysyłania" << std::endl;
	}
	
	
	simxAddStatusbarMessage(clientID,"Witajcie z Vreportera!", simx_opmode_oneshot);
	int pingTime;
	simxGetPingTime(clientID,&pingTime);
	
	
	simxFinish(clientID);
	return 0;
}
