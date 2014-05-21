#pragma once
#include <list> 

class MapNode
{
public:
	MapNode();
	MapNode(bool blocked, int cost, int x, int y);
	bool isBlocked();
	bool getCost();
	void display();
	std::pair<int, int> getXY();
private:
	bool blocked;
	int cost;
	int x, y;
};

