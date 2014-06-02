#pragma once
#include <list>
#include "MapNode.h"
#include "Pathfinder.h"
#include "Cursor.h"
#include <boost/graph/adjacency_list.hpp>
#include <SDL.h>
#include <SDL_image.h>
#include <map>
#include <set>
#include <vector>


typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, MapNode, double> MapNodeGraph;
typedef boost::graph_traits<MapNodeGraph>::vertex_descriptor MapVertex;
typedef boost::graph_traits<MapNodeGraph>::edge_descriptor MapEdge;

class Map
{
public:
	Map();
	Map(int width, int height);
	MapVertex mv;

	int getWidth();
	int getHeight();
	void display();
	bool moveCursor(int x, int y);

	enum Direction{UP,DOWN,LEFT,RIGHT};
private:
	int width, height;
	MapNodeGraph mapNodeGraph;
	MapNodeGraph abstractL2MapNodeGraph;
	std::map<coords, MapVertex> XYToGraphNodeMap;
	std::map<coords, MapVertex> XYToabstractL2GraphSectorCornerNodesMap;
	SDL_Surface *background;
	Cursor cursor;
	Pathfinder pathfindingComponent;

	struct Sector {
		coords upperLeftPoint, lowerRightPoint;
		std::set<MapVertex> vertexSet;

		Sector(coords ULPoint, coords LRPoint) {
			upperLeftPoint = ULPoint;
			lowerRightPoint = LRPoint;
		}
	};
	std::vector<std::vector<Sector>> sectors;

	void makeEmptyMap();

	//move to Pathfinder class
	void abstractMap();
	void buildEntrances(Sector& s1, Sector& s2, Direction s2Direction);
	bool isInSectorCorner(const Sector& s, coords co);
	MapVertex addAbstractGraphVertex(Sector& s, coords co);
	void addAbstractGraphNeighborEdge(MapVertex v1, MapVertex v2);
	void connectSectorVertices(Sector& s);
	std::list<MapVertex> findPathHierarchical(MapVertex v1, MapVertex v2);
};