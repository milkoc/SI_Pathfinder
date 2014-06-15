#pragma once
#include <SDL.h>
#include <SDL_image.h>
#include <stdio.h>
#include "map.h"
#include "SeekerBot.h"

struct tableCell
{
	int x, y;
	Uint32 color;
};

enum Tryb
{
	zmien_cel,
	dodaj,
	usun,
};

class App
{

public:
	//Screen dimension constants
	static const int SCREEN_WIDTH = 800;
	static const int SCREEN_HEIGHT = 600;

	struct Win
	{
		SDL_Window* gWindow;
		SDL_Surface* gScreenSurface;

		Win()
		{
			gWindow = NULL;
			gScreenSurface = NULL;
		}
	};

	App();
	void start();
private:
	Win mainWin;
	SDL_Surface* bgIMG;
	SDL_Surface* tileIMG;

	Map map;

	SDL_Event e;
	bool quit;

	bool init();
	bool initSDL();
	bool loadMedia();
	void run();
	void close();

	void handleEvents();
	void updateDisplay();

	Tryb tryb;
	SeekerBot seekerBot;
	static const int mapLength = 100;
	int cellSize;
	tableCell tableMap[mapLength][mapLength];			// tablica stworzona z grafu
	void createTableMap();
	void drawMap();
	void mouseClick(int x, int y);
};