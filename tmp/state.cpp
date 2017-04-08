#include "state.hpp" 
void VelsState::set(sf::Keyboard::Key key, bool pressed)
{
	mainMutex.lock();
	setDef(key, pressed);
	mainMutex.unlock();
}

double VelsState::get(int wheel)
{
	double ret;
	mainMutex.lock();
	ret = getDef(wheel);
	mainMutex.unlock();
	return ret;
}

void VelsState::reset()
{
	mainMutex.lock();
	resetDef();
	mainMutex.unlock();
}

void VelsState::update()
{
	mainMutex.lock();
	updateDef();
	mainMutex.unlock();
}
