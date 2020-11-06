#include "simulation.h"

void CSimulation::UpdateSimulation(uint64_t elapsed_ns)
{
	printf("Updating model for the past %lu ns\n", elapsed_ns);

	// update the sim items
	for (sim_list::iterator i = sim_objects.begin(); i != sim_objects.end(); i++)
	{
		(*i)->Update(elapsed_ns);
	}
}

void CSimulation::Draw(SDL_Surface* surface)
{
	printf("Starting to draw\n");

	// draw the background
	SDL_FillRect(surface, NULL, SDL_MapRGB( surface->format, 0x00, 0xA0, 0xA0 ) );

	// draw the UI items
	for (sim_list::iterator i = sim_objects.begin(); i != sim_objects.end(); i++)
	{
		(*i)->Draw(surface);
	}
}

