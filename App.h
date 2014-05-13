#pragma once
#include <SDL.h>
#include <SDL_image.h>
#include <stdio.h>
#include "map.h"

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
	SDL_Surface* bgIMG = NULL;
	SDL_Surface* tileIMG = NULL;

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
};