#include <SDL2/SDL.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <getopt.h>
#include <list>

#include "simulation.h"
#include "mysim.h"

using namespace std;

#define TIME_DIFFERENCE(_start, _end) \
    ((_end.tv_sec + _end.tv_nsec / 1.0e9) - \
    (_start.tv_sec + _start.tv_nsec / 1.0e9))

#define TIME_ELAPSED_NS(_start, _end) \
    ((_end.tv_sec * 1.0e9 + _end.tv_nsec) - \
    (_start.tv_sec * 1.0e9 + _start.tv_nsec))

#define TARGET_FRAMERATE 30

#ifndef VERSION
#error You must define the program version in the 'VERSION' symbol. Try using -DVERSION=<x>
#endif

static struct option options[] =
{
  {"rate", required_argument, 0, 'r'},
  {"tracefile", required_argument, 0, 't'},
  {0, no_argument, 0, 0}
};

program_state state;

static void DispatchInput(CSimulation *sim)
{
	SDL_Event event;
	if (SDL_PollEvent(&event))
	{
		sim->HandleEvent(&event);
	}
}

static void usage(void)
{
	printf("Robot Ballistic Simulator v%u\n", VERSION);
	printf("Joel Nider <joel@ece.ubc.ca>\n");
	printf("usage:\n");
	printf("h: Help - this screen\n");
	printf("r <int>: 'realtime' mode, causes the simulation to progress independently from the wall clock. Update time is in ns.\n");
	printf("t <string>: trace log file name. Output the sensor log to this file, in CSV format\n");
}

int main(int argc, char* argv[])
{
	SDL_Window* window = NULL;
	SDL_Renderer* renderer = NULL;
	SDL_Surface* surface = NULL;
	SDL_DisplayMode mode;
	double frame_delay = 0.003;
	state.realtime = true;
	state.update_rate = 80;
	state.ui_visible = true;
	state.trace_filename = NULL;

	int c;
	while (1)
	{
		c = getopt_long (argc, argv, "hr:t:", options, 0);
		if (c == -1)
		break;

		switch (c)
		{
		case 'h':
			usage();
			return 0;
		case 'r':
			state.realtime = false;
			state.update_rate = atoll(optarg);
			break;
		case 't':
			state.trace_filename = strdup(optarg);
			break;
		}
	}

	//Initialize SDL
	if( SDL_Init( SDL_INIT_VIDEO ) < 0 )
	{
		printf( "SDL could not initialize! SDL_Error: %s\n", SDL_GetError() );
		return -1;
	}

	if(TTF_Init()==-1)
	{
		printf("TTF_Init: %s\n", TTF_GetError());
		exit(2);
	}

	// check current display mode on display 0
	SDL_GetCurrentDisplayMode(0, &mode);

	//Create window
	window = SDL_CreateWindow("515 Simulator", SDL_WINDOWPOS_CENTERED,
		SDL_WINDOWPOS_CENTERED, mode.w, mode.h, SDL_WINDOW_FULLSCREEN);
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
	renderer = SDL_CreateSoftwareRenderer(surface);

	struct timespec prev, now, start, fr_start;
	unsigned int frame_count;

	if (!state.trace_filename)
		state.trace_filename = strdup("trace.out");

	while (!state.quit)
	{
		frame_count = 0;
		state.sim_running = SIM_STATE_RUNNING;
		state.fps_target = 30;
		state.total_time = 0;
		state.trials++;

		// Initialize the objects to be simulated
		CMySimulation sim1;
		if (!sim1.Initialize(&state, w, h))
			state.sim_running = SIM_STATE_STOPPED;

		clock_gettime(CLOCK_MONOTONIC, &start);
		fr_start = start;
		now = start;
		while (state.sim_running != SIM_STATE_STOPPED)
		{
			prev = now;
			clock_gettime(CLOCK_MONOTONIC, &now);

			// check for input
			DispatchInput(&sim1);

			// update the simulation
			if (state.sim_running == SIM_STATE_RUNNING)
			{
				uint64_t elapsed;
				if (!state.realtime)
					elapsed = state.update_rate;
				else
					elapsed = TIME_ELAPSED_NS(prev, now);
				state.total_time += elapsed;
				sim1.UpdateSimulation(state.total_time, elapsed);
			}

			// update the UI
			if (TIME_DIFFERENCE(start, now) > frame_delay)
			{
				frame_count++;

				sim1.Draw(renderer);
				SDL_UpdateWindowSurface(window);

				clock_gettime(CLOCK_MONOTONIC, &start);
			}

			// calculate frame rate
			if (TIME_DIFFERENCE(fr_start, now) > 1)
			{
				//printf("FPS: %u\n", frame_count);
				if (frame_count > state.fps_target)
					frame_delay *= 1.2;
				if (frame_count < state.fps_target)
					frame_delay *= 0.8;
				frame_count = 0;
				fr_start = now;

				//printf("[%lu]: ", state.total_time);
			}
		}
	}

	if (state.trace_filename)
		free(state.trace_filename);

	//Destroy window
	SDL_DestroyWindow(window);

	//Quit SDL subsystems
	SDL_Quit();

	return 0;
}
