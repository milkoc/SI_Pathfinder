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

				if (openSet.find(neighbourVertex) == openSet.end() || tempCost < baseCostMap[neighbourVertex]) {
					parentMap[neighbourVertex] = currentVertex;
					baseCostMap[neighbourVertex] = tempCost;
					adjustedCostMap[neighbourVertex] = tempCost + hueristicEstimate(graph, neighbourVertex, goalVertex);
					openSet.insert(neighbourVertex);
				}
			}
		}
	}

	std::list<MapVertex> path;
	return path; //FAIL
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

std::list<MapVertex> Pathfinder::updatePath(MapNodeGraph& graph, std::list<MapVertex> path, double(*hueristicEstimate)(MapNodeGraph&, MapVertex&, MapVertex&)) {
	std::list<MapVertex> newPath;
	std::list<MapVertex>::iterator it1;
	it1 = path.begin();

	while (it1 != path.end()) {
		std::list<MapVertex>::iterator it2 = it1, it3 = it1;
		++it3;

		while (it3 != path.end() && !boost::edge(*it2, *it3, graph).second) { //seek earliest vertex still connected to its successor
			++it2;
			++it3;
		}
		if (it1 != it2 && it2 != path.end()) {	//reconstruct subpath if vertices vere skipped
			std::list<MapVertex> subpath = findPath(graph, *it1, *it2, hueristicEstimate);
			newPath.splice(newPath.end(), subpath);		
			it1 = it2;
		}
		else {
			newPath.push_back(*it1);
			++it1;
		}			
	}
	return newPath;
}

std::list<MapVertex> Pathfinder::updatePathGoal(MapNodeGraph& graph, std::list<MapVertex> path, MapVertex& newGoal, double(*hueristicEstimate)(MapNodeGraph&, MapVertex&, MapVertex&)) {
	double currPathCost = 0;
	std::map<double, std::list<MapVertex>::iterator> estimatedCostMap;	//sort (path cost -> last vertex) pairs
	std::list<MapVertex>::iterator it1 = path.begin();

	while (it1 != path.end()) {		//estimate lenghts of possible new paths
		double estimatedCost = currPathCost + hueristicEstimate(graph, *it1, newGoal);
		estimatedCostMap[estimatedCost] = it1;

		std::list<MapVertex>::iterator it2 = it1;
		++it2;
		if (it2 != path.end()){
			currPathCost += graph[boost::edge(*it1, *it2, graph).first];	//assume that path is still passable
		}
		++it1;
	}

	std::list<MapVertex>::iterator lastKeptNodeit = estimatedCostMap.begin()->second;	//shortest estimated new path = path to this node + subpath to the new goal
	std::list<MapVertex> newPath;
	newPath.splice(newPath.begin(), path, path.begin(), lastKeptNodeit);	//move kept part of the old path into the new path
	std::list<MapVertex> newGoalSubpath = findPath(graph, *lastKeptNodeit, newGoal, hueristicEstimate);
	newGoalSubpath.pop_front();	//Delete the first element, it's lastKeptNode
	newPath.splice(newPath.end(), newGoalSubpath);	//add new goal subpath to the new path

	return newPath;
}

double Pathfinder::pathCost(MapNodeGraph& graph, std::list<MapVertex> path) {
	double currPathCost = 0;
	std::list<MapVertex>::iterator it1 = path.begin();

	while (it1 != path.end()) {	
		std::list<MapVertex>::iterator it2 = it1;
		++it2;
		if (it2 != path.end()){
			currPathCost += graph[boost::edge(*it1, *it2, graph).first];	//assume that path is still passable
		}
		++it1;
	}

	return currPathCost;
}

