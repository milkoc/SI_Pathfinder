#pragma once
#include "Pathfinder.h"
#include "Map.h"
class SeekerBot
{
public:
	SeekerBot();
	~SeekerBot();
private:
	Pathfinder pathfindingComponent;
	std::list<MapVertex> Path;

	friend class App;
};

