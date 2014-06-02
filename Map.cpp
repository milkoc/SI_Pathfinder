#include "map.h"

Map::Map()
{
}

Map::Map(int width, int height) : width(width), height(height)
{
	makeEmptyMap();
}

void Map::makeEmptyMap()
{

}

int Map::getWidth()
{
	return width;
}
int Map::getHeight()
{
	return height;
}

bool Map::moveCursor(int x, int y) {
	return cursor.move(x, y);
}

void Map::abstractMap() {
	const int sectorSide = 10;

	int xSectorsNum = (width % sectorSide) ? width / sectorSide + 1 : width / sectorSide;
	int ySectorsNum = (height % sectorSide) ? height / sectorSide + 1 : height / sectorSide;
	sectors = std::vector<std::vector<Sector>>(xSectorsNum, std::vector<Sector>(ySectorsNum, Sector(coords(0, 0), coords(0, 0))));

	for (int x = 0; x < xSectorsNum; ++x) {	//determine sector boundaries
		for (int y = 0; y < ySectorsNum; ++y) {
			coords ULpoint(x * sectorSide, y * sectorSide); //upper left
			sectors[x][y].upperLeftPoint = ULpoint;
			coords LRpoint = ULpoint;//lower right
			LRpoint.first = LRpoint.first + sectorSide >= width ? width - 1 : LRpoint.first + sectorSide;	//adjust lr x
			LRpoint.second = LRpoint.second + sectorSide >= height ? height - 1 : LRpoint.second + sectorSide;	//adjust lr y

			sectors[x][y].upperLeftPoint = ULpoint;
			sectors[x][y].lowerRightPoint = LRpoint;
		}
	}

	for (int x = 0; x < xSectorsNum; ++x) {	//build entrances to lower and right sectors
		for (int y = 0; y < ySectorsNum; ++y) {
			if (x + 1 < xSectorsNum) {	//if there is right sector
				buildEntrances(sectors[x][y], sectors[x+1][y], RIGHT);
			}
			if (y + 1 < ySectorsNum) {	//if there is lower sector
				buildEntrances(sectors[x][y], sectors[x][y+1], DOWN);
			}
		}
	}
}

void Map::buildEntrances(Sector& s1, Sector& s2, Direction s2Direction) {	//only right or down direction; needs refactoring
	const int entranceWidthThreshold = 6; //if entrance is less wide, put transition node in the middle, else put 2 trans nodes at both ends

	switch (s2Direction) {
	case DOWN:
		int x = s1.upperLeftPoint.first;
		int y1 = s1.lowerRightPoint.second;
		int xLimit = s1.lowerRightPoint.first;
		int y2 = y1 + 1;

		while (x <= xLimit) {
			if ( (XYToGraphNodeMap.find(coords(x, y1)) != XYToGraphNodeMap.end()) && (XYToGraphNodeMap.find(coords(x, y2)) != XYToGraphNodeMap.end()) ) {	//entrance beginning foung
				int entranceX1 = x, entranceX2 = x;
				++x;
				while ((x <= xLimit) && (XYToGraphNodeMap.find(coords(x, y1)) != XYToGraphNodeMap.end()) && (XYToGraphNodeMap.find(coords(x, y2)) != XYToGraphNodeMap.end())) {	//assume adjacent nodes are connected
					++x;
				}
				entranceX2 = x - 1;
				int entLen = (entranceX2 - entranceX1) + 1;

				//determine transition nodes and insert them into abstract graph
				if (entLen >= entranceWidthThreshold && entLen > 1) {	//length above or equal threshold, add a node at each end for both sectors
					//first transition nodes pair
					coords v1aCoords(entranceX1, y1), v1bCoords(entranceX1, y2);
					MapVertex v1a = addAbstractGraphVertex(s1, v1aCoords), v1b = addAbstractGraphVertex(s2, v1bCoords);
					addAbstractGraphNeighborEdge(v1a, v1b); addAbstractGraphNeighborEdge(v1b, v1a);
					//second transition nodes pair
					coords v2aCoords(entranceX2, y1), v2bCoords(entranceX2, y2);
					MapVertex v2a =  addAbstractGraphVertex(s1, v2aCoords), v2b = addAbstractGraphVertex(s2, v2bCoords);
					addAbstractGraphNeighborEdge(v2a, v2b); addAbstractGraphNeighborEdge(v2b, v2a);
				}
				else {	//add one node at the middle for both sectors
					int midX = entranceX1 + (entLen / 2);
					coords v1aCoords(midX, y1), v1bCoords(midX, y2);
					MapVertex v1a = addAbstractGraphVertex(s1, v1aCoords), v1b = addAbstractGraphVertex(s2, v1bCoords);
					addAbstractGraphNeighborEdge(v1a, v1b); addAbstractGraphNeighborEdge(v1b, v1a);
				}

			}
			++x;
		}

		break;
	case RIGHT:

		break;
	default:
		return;
		break;
	}

}

bool Map::isInSectorCorner (const Sector& s, coords co) {
	return co == s.lowerRightPoint || co == s.upperLeftPoint || (co.first == s.upperLeftPoint.first && co.second == s.lowerRightPoint.second) || (co.second == s.upperLeftPoint.second && co.first == s.lowerRightPoint.first);
}

MapVertex Map::addAbstractGraphVertex(Sector& s, coords co) {
	std::map<coords, MapVertex>::iterator cornerNodesMapIter = XYToabstractL2GraphSectorCornerNodesMap.find(co);
	if (!isInSectorCorner(s, co) || cornerNodesMapIter == XYToabstractL2GraphSectorCornerNodesMap.end()) {	//add corner nodes only once
		MapNode node1 = mapNodeGraph[XYToGraphNodeMap.find(co)->second];
		MapVertex v1 = boost::add_vertex(node1, abstractL2MapNodeGraph);
		if (isInSectorCorner(s, co)) {
			XYToabstractL2GraphSectorCornerNodesMap[co] = v1;
		}

		return v1;
	}
	else {
		return cornerNodesMapIter->second;
	}
}

void Map::addAbstractGraphNeighborEdge(MapVertex v1, MapVertex v2) {
	MapEdge e; bool b;
	boost::tie(e, b) = boost::add_edge(v1, v2, abstractL2MapNodeGraph);

	coords co1 = abstractL2MapNodeGraph[v1].getXY(), co2 = abstractL2MapNodeGraph[v2].getXY();
	double cost = mapNodeGraph[boost::edge(XYToGraphNodeMap.find(co1)->second, XYToGraphNodeMap.find(co2)->second, mapNodeGraph).first];
	abstractL2MapNodeGraph[e] = cost;
}

void Map::connectSectorVertices(Sector& s) {	//connect all sector vertices with a passable path between them //PATHFINDING INSIDE SECTOR NEEDED
	/*
	for (std::set<MapVertex>::iterator it1 = s.vertexSet.begin(); it1 != s.vertexSet.end(); ++it1) {
		coords co1 = abstractL2MapNodeGraph[*it1].getXY();
		MapVertex fullGV1 = XYToGraphNodeMap.find(co1)->second;	//vertex 1 from the full graph
		for (std::set<MapVertex>::iterator it2 = s.vertexSet.begin(); it2 != s.vertexSet.end(); ++it2) {
			if (*it1 != *it2) {
				coords co2 = abstractL2MapNodeGraph[*it2].getXY();
				MapVertex fullGV2 = XYToGraphNodeMap.find(co1)->second; //vertex 2 from the full graph
				std::list<MapVertex> path = pathfindingComponent.findPath(mapNodeGraph, fullGV1, fullGV2, &Pathfinder::manhattanDistanceHeuristic); //PATHFINDING INSIDE SECTOR NEEDED
				if (path.size() > 0) {	//if path exists
					MapEdge e; bool b;
					boost::tie(e, b) = boost::add_edge(*it1, *it2, abstractL2MapNodeGraph);	//add edge
					abstractL2MapNodeGraph[e] = pathfindingComponent.pathCost(mapNodeGraph, path);
				}
			}
		}
	}
	*/
}

std::list<MapVertex> Map::findPathHierarchical(MapVertex v1, MapVertex v2) {
	std::list<MapVertex> path;
	MapNodeGraph abstractGraphCp = abstractL2MapNodeGraph;
	//add both vertices to the copy, connect them with entrances in their sector and then search the copy
	return path; //fail
}
