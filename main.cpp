#include <SDL2/SDL.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <list>

using namespace std;

#define TIME_DIFFERENCE(_start, _end) \
    ((_end.tv_sec + _end.tv_nsec / 1.0e9) - \
    (_start.tv_sec + _start.tv_nsec / 1.0e9))

#define TIME_ELAPSED_NS(_start, _end) \
    ((_end.tv_sec * 1.0e9 + _end.tv_nsec) - \
    (_start.tv_sec * 1.0e9 + _start.tv_nsec))

#define TARGET_FRAMERATE 30

class ui_object
{
public:
	virtual void Draw(SDL_Surface* s) = 0;
};

class sim_object
{
public:
	sim_object(uint64_t x, uint64_t y) :
		m_pos_x(x), m_pos_y(y), m_velocity_x(0), m_velocity_y(0) {}
	virtual void Update(uint64_t nsec) = 0;

protected:
	uint32_t m_color;
	double m_pos_x;
	double m_pos_y;
	double m_velocity_x;
	double m_velocity_y;
};

class robot : public ui_object, public sim_object
{
public:
	robot() : sim_object(0,50) { m_rect.w = 50; m_rect.h=20;  m_velocity_x=1; }
	~robot() {};
	void Draw(SDL_Surface* s);
	void Update(uint64_t nsec);
private:
	SDL_Rect m_rect;
};

typedef std::list<ui_object*> ui_list;
typedef std::list<sim_object*> sim_list;

ui_list ui_objects;
sim_list sim_objects;

void robot::Draw(SDL_Surface* s)
{
	m_rect.x = m_pos_x * 0.01;
	m_rect.y = m_pos_y * 0.01;
	printf("%s x=%u y=%u\n", __PRETTY_FUNCTION__, m_rect.x, m_rect.y); // __METHOD_NAME__
	SDL_FillRect(s, &m_rect, SDL_MapRGB(s->format, 0x1F, 0x02, 0x20));
}

void robot::Update(uint64_t nsec)
{
	//printf("%s\n", __PRETTY_FUNCTION__);
	// update position based on velocity
	m_pos_x += m_velocity_x;
	m_pos_y += m_velocity_y;
	printf("X: %f Y: %f\n", m_pos_x, m_pos_y);
}

static void UpdateSimulation(uint64_t elapsed_ns)
{
	//printf("Updating model for the past %lu ns\n", elapsed_ns);

	// update the sim items
	for (sim_list::iterator i = sim_objects.begin(); i != sim_objects.end(); i++)
	{
		(*i)->Update(elapsed_ns);
	}
}

static void Draw(SDL_Surface* s)
{
	//printf("Starting to draw\n");

	// draw the background
	SDL_FillRect(s, NULL, SDL_MapRGB( s->format, 0x00, 0xA0, 0xA0 ) );

	// draw the UI items
	for (ui_list::iterator i = ui_objects.begin(); i != ui_objects.end(); i++)
	{
		(*i)->Draw(s);
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
	unsigned int delay = 5000;
	struct timespec prev, now, start;

	// create a robot 
	robot r;

	//add it to the ui list so it will be drawn
	ui_objects.push_back(&r);

	//add it to the simulation list so it will be updated
	sim_objects.push_back(&r);

	clock_gettime(CLOCK_MONOTONIC, &start);
	while (sim_running)
	{
		prev = now;
		clock_gettime(CLOCK_MONOTONIC, &now);

		// check for input

		// update the simulation
		UpdateSimulation(TIME_ELAPSED_NS(prev, now));

		// calculate frame rate
		if (TIME_DIFFERENCE(start, now) > frame_delay)
		{
			frame_count++;

			// update the UI
			Draw(surface);
			SDL_UpdateWindowSurface(window);

/*
			if (frame_count > TARGET_FRAMERATE)
			{
				delay *= 1.1;
				printf("Set delay to %u\n", delay);
			}

			if (frame_count < TARGET_FRAMERATE)
			{
				delay *= 0.9;
				printf("Set delay to %u\n", delay);
			}
			frame_count = 0;
*/

			clock_gettime(CLOCK_MONOTONIC, &start);
			count++;
		}

		//usleep(delay);

		if (frame_count == 300)
			sim_running = 0;
	}

	//Destroy window
	SDL_DestroyWindow( window );

	//Quit SDL subsystems
	SDL_Quit();

	return 0;
}

