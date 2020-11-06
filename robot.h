#ifndef _ROBOT__H
#define _ROBOT__H

#include <SDL2/SDL.h>
#include "simulation.h"

class robot : public sim_object
{
public:
	robot() : sim_object(0,50) { m_rect.w = 50; m_rect.h=20;  m_velocity_x=0.0005; }
	~robot() {};
	void Draw(SDL_Surface* s);
	void Update(uint64_t nsec);
private:
	SDL_Rect m_rect;
};

#endif // _ROBOT__H
