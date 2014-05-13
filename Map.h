#pragma once
#include <list>
#include "MapNode.h"
#include "Cursor.h"
#include <boost/graph/adjacency_list.hpp>
#include <SDL.h>
#include <SDL_image.h>

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, MapNode> MapNodeGraph;
typedef boost::graph_traits<MapNodeGraph>::vertex_descriptor MapVertex;
typedef boost::graph_traits<MapNodeGraph>::edge_descriptor MapEdge;

class Map
{
public:
	Map();
	Map(int width, int height);

	int getWidth();
	int getHeight();
	void display();
	bool moveCursor(int x, int y);
private:
	int width, height;
	MapNodeGraph mapNodeGraph;
	SDL_Surface *background;
	Cursor cursor;

	void makeEmptyMap();
};