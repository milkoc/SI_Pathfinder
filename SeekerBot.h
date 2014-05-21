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
};

