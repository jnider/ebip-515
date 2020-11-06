#ifndef _SIMULATION__H
#define  _SIMULATION__H

#include <SDL2/SDL.h>
#include <list>

class sim_object
{
public:
	sim_object(uint64_t x, uint64_t y) :
		m_pos_x(x), m_pos_y(y), m_velocity_x(0), m_velocity_y(0) {}
	virtual void Update(uint64_t nsec) = 0;
	virtual void Draw(SDL_Surface* s) = 0;

protected:
	uint32_t m_color;
	double m_pos_x;
	double m_pos_y;
	double m_velocity_x;
	double m_velocity_y;
};

typedef std::list<sim_object*> sim_list;

class CSimulation
{
public:
	virtual bool Initialize() = 0;
	virtual void UpdateSimulation(uint64_t elapsed_ns);
	virtual void Draw(SDL_Surface* s);

public:
	sim_list sim_objects;
};

#endif // _SIMULATION__H
