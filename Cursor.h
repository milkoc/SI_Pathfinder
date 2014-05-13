#pragma once
class Cursor
{
public:
	Cursor();
	int getX();
	int getY();
	bool move(int x, int y);
	void display();
private:
	int x, y;
	int maxX, maxY;
};

