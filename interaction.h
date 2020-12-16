#ifndef _INTERACTION__H
#define _INTERACTION__H

#include <stdio.h>
#include <list>
#include "simulation.h"

#define NUM_ENSEMBLE_MEMBERS		8
#define NUM_STATE_VARIABLES		5
#define MAX_LATENT_FUNCTIONS		1

enum
{
	STATE_VAR_PHASE,
	STATE_VAR_PHASE_VEL,
	STATE_VAR_BALL_X,
	STATE_VAR_BALL_Y,
	STATE_VAR_ROBOT_X,
};


typedef std::list<measurement*> measurement_list;

class CInteraction
{
public:
	CInteraction(double scale);
	~CInteraction();
	bool Load(FILE *f);
	void get_sample(double phase, double sample[]);
	unsigned long length() { return m_length; } // in milliseconds

private:
	double m_scale;
	unsigned long m_length;
	measurement_list m_measurements;
};

#endif // _INTERACTION__H
