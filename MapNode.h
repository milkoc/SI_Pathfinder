#pragma once
#include <list> 

typedef std::pair<int, int> coords;

class MapNode
{
public:
	MapNode();
	MapNode(bool blocked, int cost, int x, int y);
	bool isBlocked();
	bool getCost();
	void display();
	coords getXY();
private:
	bool blocked;
	int cost;
	int x, y;
};

