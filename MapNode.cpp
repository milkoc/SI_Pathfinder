#include "MapNode.h"


MapNode::MapNode()
{
	MapNode(false, 1, 0, 0);
}

MapNode::MapNode(bool blocked, int cost, int x, int y) : blocked(blocked), cost(cost), x(x), y(y)
{
}

std::pair<int, int> MapNode::getXY() {
	return std::pair<int, int>(x, y);
}