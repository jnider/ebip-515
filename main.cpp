#include <SDL2/SDL.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <list>

#include "simulation.h"
#include "robot.h"

using namespace std;

#define TIME_DIFFERENCE(_start, _end) \
    ((_end.tv_sec + _end.tv_nsec / 1.0e9) - \
    (_start.tv_sec + _start.tv_nsec / 1.0e9))

#define TIME_ELAPSED_NS(_start, _end) \
    ((_end.tv_sec * 1.0e9 + _end.tv_nsec) - \
    (_start.tv_sec * 1.0e9 + _start.tv_nsec))

#define TARGET_FRAMERATE 30

struct program_state
{
	bool sim_running; // is the simulation allowed to continue
	uint32_t fps_target; // our target frame rate
};

program_state state;

class CMySimulation : public CSimulation
{
public:
	CMySimulation() {};
	~CMySimulation() {}
	bool Initialize();

private:
	robot r;
};

bool CMySimulation::Initialize()
{
	printf("%s\n", __PRETTY_FUNCTION__);
	//add it to the simulation list so it will be updated
	sim_objects.push_back(&r);

	return true;
}

static void DispatchInput()
{
	SDL_Event event;
	if (SDL_PollEvent(&event))
	{
		switch(event.type)
		{
		case SDL_KEYDOWN:
			if (event.key.keysym.sym == SDLK_ESCAPE)
				state.sim_running = false;
			else
				printf("Got key %c\n", event.key.keysym.sym);
			break;
		}
	}
}

int main(int argc, char* args[])
{
	SDL_Window* window = NULL;
	SDL_Surface* surface = NULL;
	SDL_DisplayMode mode;
	double frame_delay = 0.003;

	//Initialize SDL
	if( SDL_Init( SDL_INIT_VIDEO ) < 0 )
	{
		printf( "SDL could not initialize! SDL_Error: %s\n", SDL_GetError() );
		return -1;
	}

	// check current display mode on display 0
	SDL_GetCurrentDisplayMode(0, &mode);

	//Create window
	window = SDL_CreateWindow("515 Simulator", SDL_WINDOWPOS_CENTERED,
		SDL_WINDOWPOS_CENTERED, mode.w, mode.h, SDL_WINDOW_OPENGL | SDL_WINDOW_FULLSCREEN);
	if(!window)
	{
		printf( "Window could not be created! SDL_Error: %s\n", SDL_GetError() );
		return -2;
	}

	int w, h;
	SDL_GetWindowSize(window, &w, &h);
	printf("Your resolution is %ix%i\n", w, h);

	//Get window surface
	surface = SDL_GetWindowSurface(window);

	unsigned int frame_count = 0;
	unsigned int count = 0;
	unsigned int sim_running = 1;
	struct timespec prev, now, start, fr_start;
	state.sim_running = true;
	state.fps_target = 30;

	// Initialize the objects to be simulated
	CMySimulation sim1;
	sim1.Initialize();

	clock_gettime(CLOCK_MONOTONIC, &start);
	fr_start = start;
	now = start;
	while (state.sim_running)
	{
		prev = now;
		clock_gettime(CLOCK_MONOTONIC, &now);

		// check for input
		DispatchInput();

		// update the simulation
		sim1.UpdateSimulation(TIME_ELAPSED_NS(prev, now));

		if (TIME_DIFFERENCE(start, now) > frame_delay)
		{
			frame_count++;

			// update the UI
			sim1.Draw(surface);
			SDL_UpdateWindowSurface(window);

			clock_gettime(CLOCK_MONOTONIC, &start);
			count++;
		}

		// calculate frame rate
		if (TIME_DIFFERENCE(fr_start, now) > 1)
		{
			printf("FPS: %u\n", frame_count);
			if (frame_count > state.fps_target)
				frame_delay *= 1.1;
			if (frame_count < state.fps_target)
				frame_delay *= 0.9;
			frame_count = 0;
			fr_start = now;
		}
	}

	//Destroy window
	SDL_DestroyWindow( window );

	//Quit SDL subsystems
	SDL_Quit();

	return 0;
}
