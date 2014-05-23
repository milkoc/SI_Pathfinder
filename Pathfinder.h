#pragma once
#include <list>
#include <set>
#include <map>
#include <boost/bimap.hpp>
#include "Map.h"

class Pathfinder
{
public:
	Pathfinder();
	std::list<MapVertex> findPath(MapNodeGraph& graph, MapVertex& startVertex, MapVertex& goalVertex, double(*hueristicEstimate)(MapNodeGraph&, MapVertex&, MapVertex&));
	static double manhattanDistanceHeuristic(MapNodeGraph& graph, MapVertex& vertexA, MapVertex& vertexB);
	std::list<MapVertex> updatePath(MapNodeGraph& graph, std::list<MapVertex> path, double(*hueristicEstimate)(MapNodeGraph&, MapVertex&, MapVertex&));
	std::list<MapVertex> updatePathGoal(MapNodeGraph& graph, std::list<MapVertex> path, MapVertex& newGoal, double(*hueristicEstimate)(MapNodeGraph&, MapVertex&, MapVertex&));
private:
	std::list<MapVertex> constructPath(std::map<MapVertex, MapVertex> parentMap, MapVertex& currentVertex);
};

