#include "Pathfinder.h"

Pathfinder::Pathfinder()
{
}

std::list<MapVertex> Pathfinder::findPath(MapNodeGraph& graph, MapVertex& startVertex, MapVertex& goalVertex, double(*hueristicEstimate)(MapNodeGraph&, MapVertex&, MapVertex&)) {
	typedef std::set<MapVertex> vertexSet;
	vertexSet openSet, closedSet;
	std::map<MapVertex, MapVertex> parentMap;
	std::map<MapVertex, double> baseCostMap, adjustedCostMap;

	baseCostMap[startVertex] = 0;
	adjustedCostMap[startVertex] = hueristicEstimate(graph, startVertex, goalVertex);
	openSet.insert(startVertex);

	while (!openSet.empty()) {
		MapVertex currentVertex = *(openSet.begin());
		double currCost = adjustedCostMap[*(openSet.begin())];
		for (vertexSet::iterator it = openSet.begin(); it != openSet.end(); ++it) {
			double cost = adjustedCostMap[*it];
			if (cost < currCost) {
				currentVertex = *it;
				currCost = cost;
			}
		}

		if (currentVertex == goalVertex) {
			return constructPath(parentMap, goalVertex);
		}
		openSet.erase(currentVertex);
		closedSet.insert(currentVertex);

		MapNodeGraph::adjacency_iterator neighbourIt, neighbourEnd;
		boost::tie(neighbourIt, neighbourEnd) = boost::adjacent_vertices(currentVertex, graph);
		for (; neighbourIt != neighbourEnd; ++neighbourIt) {
			MapVertex neighbourVertex = *neighbourIt;
			if (closedSet.find(neighbourVertex) == closedSet.end()) {
				MapEdge edge = boost::edge(currentVertex, neighbourVertex, graph).first;
				double tempCost = baseCostMap[currentVertex] + graph[edge];

				if (openSet.find(neighbourVertex) == closedSet.end() || tempCost < baseCostMap[neighbourVertex]) {
					parentMap[neighbourVertex] = currentVertex;
					baseCostMap[neighbourVertex] = tempCost;
					adjustedCostMap[neighbourVertex] = tempCost + hueristicEstimate(graph, neighbourVertex, goalVertex);
					openSet.insert(neighbourVertex);
				}
			}
		}
	}

	return std::list<MapVertex>(); //FAIL
}

std::list<MapVertex> Pathfinder::constructPath(std::map<MapVertex, MapVertex> parentMap, MapVertex& currentVertex) {
	std::list<MapVertex> path;
	if (parentMap.find(currentVertex) != parentMap.end()) {
		path = constructPath(parentMap, parentMap[currentVertex]);
	}
	path.push_back(currentVertex);
	return path;
}

double Pathfinder::manhattanDistanceHeuristic(MapNodeGraph& graph, MapVertex& vertexA, MapVertex& vertexB) {
	//return sqrt(pow(graph[vertexA].getXY().first - graph[vertexB].getXY().first, 2) + pow(graph[vertexA].getXY().second - graph[vertexB].getXY().second, 2));
	return abs(graph[vertexA].getXY().first - graph[vertexB].getXY().first) + abs(graph[vertexA].getXY().second - graph[vertexB].getXY().second);
}