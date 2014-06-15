#include "App.h"

App::App() : map(100, 100)
{
	bgIMG = NULL;
	tileIMG = NULL;
	cellSize = SCREEN_WIDTH / mapLength;
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
	createTableMap();
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
	//seekerBot.Path = seekerBot.pathfindingComponent.findPath(map.mapNodeGraph, map.XYToGraphNodeMap[coords(0,0)], map.XYToGraphNodeMap[coords(10,10)], Pathfinder::manhattanDistanceHeuristic);
	seekerBot.Path = map.findPath(map.mapNodeGraph, map.XYToGraphNodeMap[coords(0, 0)], map.XYToGraphNodeMap[coords(10, 10)], Map::manhattanDistanceHeuristic);

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
		else if (e.button.type == SDL_MOUSEBUTTONDOWN)
		{
			if (e.button.button == SDL_BUTTON_LEFT)
				tryb = dodaj;
			if (e.button.button == SDL_BUTTON_RIGHT)
				tryb = usun;
			App::mouseClick(e.button.x, e.button.y);
		}

		//User presses a key
		/*
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
		}*/
	}
}

void App::updateDisplay()
{
	//Apply the image
	SDL_BlitSurface(bgIMG, NULL, mainWin.gScreenSurface, NULL);
	drawMap();
	//Update the surface
	SDL_UpdateWindowSurface(mainWin.gWindow);
}

void App::createTableMap()
{
	for (int i = 0; i<mapLength; i++)
	{
		for (int j = 0; j<mapLength; j++)
		{
			tableMap[i][j].x = i*cellSize;
			tableMap[i][j].y = j*cellSize;
			tableMap[i][j].color = 0x78AB46;
		}
	}
}

void App::drawMap()
{
	int bpp = mainWin.gScreenSurface->format->BytesPerPixel;

	MapVertex v;
	int x, y, i, j, x1, y1;
	Uint8 *p;

	// kolorowanie kratek
	for (x = 0; x < SCREEN_HEIGHT; x += cellSize)
	{
		for (y = 0; y < SCREEN_HEIGHT; y += cellSize)
		{
			for (i = x + 1; i < x + cellSize - 1; i++)
			{
				for (j = y + 1; j < y + cellSize - 1; j++)
				{
					p = (Uint8 *)mainWin.gScreenSurface->pixels + j * mainWin.gScreenSurface->pitch + i * bpp;
					*(Uint32 *)p = tableMap[x / cellSize][y / cellSize].color;
				}
			}
		}
	}

	// kolorowanie œcie¿ki
	if (!seekerBot.Path.empty())
	{
		for (std::list<MapVertex>::iterator iter = seekerBot.Path.begin(); iter != seekerBot.Path.end(); ++iter)
		{
			coords nodeCoords = map.mapNodeGraph[*iter].getXY();
			x1 = nodeCoords.first * cellSize;
			y1 = nodeCoords.second * cellSize;

			for (i = x1 + 1; i < x1 + cellSize - 1; i++)
			{
				for (j = y1 + 1; j < y1 + cellSize - 1; j++)
				{
					p = (Uint8 *)mainWin.gScreenSurface->pixels + j * mainWin.gScreenSurface->pitch + i * bpp;
					*(Uint32 *)p = 0xFFFFFF; 
				}
			}

		}
	}

}

void App::mouseClick(int x, int y)
{
	if (x < SCREEN_WIDTH)
	{
		x /= cellSize;
		y /= cellSize;

		if (tryb == dodaj)
		{
			if (tableMap[x][y].color != 0x78AB46)
			{
				tableMap[x][y].color = 0x78AB46;
				//tableMap[(x/cellSize)+1][(y/cellSize)].color = 0x78AB46;
				//tableMap[(x/cellSize)+1][(y/cellSize)+1].color = 0x78AB46;
				//tableMap[(x/cellSize)][(y/cellSize)+1].color = 0x78AB46;

				// dodanie wierzcho³ka
				MapNode *node = map.nodeTab[x][y];
				MapVertex v = add_vertex(*node, map.mapNodeGraph);
				map.XYToGraphNodeMap[coords(x, y)] = v;
				//EDGES
				//up
				if (map.XYToGraphNodeMap.find(coords(x, y - 1)) != map.XYToGraphNodeMap.end()) {
					add_edge(v, map.XYToGraphNodeMap[coords(x, y - 1)], 1, map.mapNodeGraph);
					add_edge(map.XYToGraphNodeMap[coords(x, y - 1)], v, 1, map.mapNodeGraph);
				}
				//down
				if (map.XYToGraphNodeMap.find(coords(x, y + 1)) != map.XYToGraphNodeMap.end()) {
					add_edge(v, map.XYToGraphNodeMap[coords(x, y + 1)], 1, map.mapNodeGraph);
					add_edge(map.XYToGraphNodeMap[coords(x, y + 1)], v, 1, map.mapNodeGraph);
				}
				//left
				if (map.XYToGraphNodeMap.find(coords(x + 1, y)) != map.XYToGraphNodeMap.end()) {
					add_edge(v, map.XYToGraphNodeMap[coords(x + 1, y)], 1, map.mapNodeGraph);
					add_edge(map.XYToGraphNodeMap[coords(x + 1, y)], v, 1, map.mapNodeGraph);
				}
				//right
				if (map.XYToGraphNodeMap.find(coords(x -1, y)) != map.XYToGraphNodeMap.end()) {
					add_edge(v, map.XYToGraphNodeMap[coords(x - 1, y)], 1, map.mapNodeGraph);
					add_edge(map.XYToGraphNodeMap[coords(x - 1, y)], v, 1, map.mapNodeGraph);
				}
			}
		}
		else
		{
			if (tableMap[x][y].color != 0x000000)
			{
				tableMap[x][y].color = 0x000000;
				//tableMap[(x/cellSize)+1][(y/cellSize)+1].color = 0x000000;
				//tableMap[(x/cellSize)+1][(y/cellSize)].color = 0x000000;
				//tableMap[(x/cellSize)][(y/cellSize)+1].color = 0x000000;

				clear_vertex(map.XYToGraphNodeMap[coords(x, y)], map.mapNodeGraph);
				remove_vertex(map.XYToGraphNodeMap[coords(x, y)], map.mapNodeGraph);
				map.XYToGraphNodeMap.erase(coords(x, y));
				map.refreshMapping(map.XYToGraphNodeMap, map.mapNodeGraph);
			}
		}

		//seekerBot.Path = seekerBot.pathfindingComponent.updatePath(map.mapNodeGraph, seekerBot.Path, Pathfinder::manhattanDistanceHeuristic);
		seekerBot.Path = map.findPath(map.mapNodeGraph, map.XYToGraphNodeMap[coords(0, 0)], map.XYToGraphNodeMap[coords(10, 10)], Map::manhattanDistanceHeuristic);
		map.pathToCoords(seekerBot.Path, map.mapNodeGraph);
	}
}

