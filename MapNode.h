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
private:
	bool blocked;
	int cost;
	int x, y;
};

