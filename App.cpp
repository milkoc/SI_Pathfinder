#include "App.h"

App::App() : map(25, 25)
{
}

void App::start()
{
	if (init())
	{
		run();
	}
	close();
}

bool App::init()
{
	return initSDL() && loadMedia();
}

bool App::initSDL()
{
	//Initialization flag
	bool success = true;

	//Initialize SDL
	if (SDL_Init(SDL_INIT_VIDEO) < 0)
	{
		printf("SDL could not initialize! SDL_Error: %s\n", SDL_GetError());
		success = false;
	}
	else
	{
		//SDL_ShowCursor(SDL_DISABLE); //Disable cursor
		//Create window
		mainWin.gWindow = SDL_CreateWindow("Pathfinder", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
		if (mainWin.gWindow == NULL)
		{
			printf("Window could not be created! SDL_Error: %s\n", SDL_GetError());
			success = false;
		}
		else
		{
			//Get window surface
			mainWin.gScreenSurface = SDL_GetWindowSurface(mainWin.gWindow);
		}
	}

	return success;
}

bool App::loadMedia()
{
	bgIMG = IMG_Load("resources/gfx/mainBG.png");
	tileIMG = IMG_Load("resources/gfx/emtyTile.png");
	return bgIMG != NULL && tileIMG != NULL;
}

void App::run()
{
	quit = false;
	while (!quit)
	{
		handleEvents();
		updateDisplay();
	}
}

void App::close()
{
	//Deallocate surface
	SDL_FreeSurface(bgIMG);
	bgIMG = NULL;

	SDL_FreeSurface(tileIMG);
	tileIMG = NULL;

	//Destroy window
	SDL_DestroyWindow(mainWin.gWindow);
	mainWin.gWindow = NULL;

	//Quit SDL subsystems
	SDL_Quit();
}

void App::handleEvents()
{
	//Handle events on queue
	while (SDL_PollEvent(&e) != 0)
	{
		//User requests quit
		if (e.type == SDL_QUIT)
		{
			quit = true;
		}
		//User presses a key
		else if (e.type == SDL_KEYDOWN)
		{
			switch (e.key.keysym.sym)
			{
			case SDLK_UP:
				map.moveCursor(0, -1);
				break;

			case SDLK_DOWN:
				map.moveCursor(0, 1);
				break;

			case SDLK_LEFT:
				map.moveCursor(-1, 0);
				break;

			case SDLK_RIGHT:
				map.moveCursor(1, 0);
				break;
			case SDLK_z:
				//make tile at the cursor position unpassable
				break;
			case SDLK_x:
				//make tile at the cursor position passable
				break;
			case SDLK_c:
				//place goal for the bots to reach at the cursor position
				break;
			default:
				break;
			}
		}
	}
}

void App::updateDisplay()
{
	//Apply the image
	SDL_BlitSurface(bgIMG, NULL, mainWin.gScreenSurface, NULL);

	//Update the surface
	SDL_UpdateWindowSurface(mainWin.gWindow);
}

