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