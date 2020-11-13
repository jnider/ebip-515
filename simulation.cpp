#include "simulation.h"

void sim_object::Draw(SDL_Renderer* renderer)
{
	SDL_Rect rect;
	rect.x = m_pos_x;
	rect.y = m_pos_y;
	rect.w = m_width;
	rect.h = m_height;
	//printf("%s x=%u y=%u\n", __PRETTY_FUNCTION__, m_rect.x, m_rect.y); // __METHOD_NAME__
	//SDL_FillRect(s, &rect, SDL_MapRGB(s->format, 0x1F, 0x02, 0x20));
	SDL_SetRenderDrawColor(renderer, 0x1F, 0x20, 0x20, 0xFF);
	SDL_RenderFillRect(renderer, &rect);

	for (int i=0; i < m_tracerPts.size(); i++)
	{
		uint32_t color = 0x00FF0000 | ((m_tracerPts.size() - i) << 25);
		filledCircleColor(renderer, m_tracerPts[i].x, m_tracerPts[i].y, 5, color);
	}
}

void sim_object::Update(uint64_t elapsed_ns)
{
	//printf("%s\n", __PRETTY_FUNCTION__);

	if (m_tracerPts.size() < m_tracerLength)
		m_tracerPts.push_back(point(-1, -1));

	if (m_tracerPts.size() > 0)
	{
		if (m_tracer_elapsed_ns > TIME_BETWEEN_TRACES)
		{
		// update tracer points
		for (int i=m_tracerPts.size()-1; i > 0; i--)
		{
			m_tracerPts[i].x = m_tracerPts[i-1].x;
			m_tracerPts[i].y = m_tracerPts[i-1].y;
		}

		m_tracerPts[0].x = m_pos_x;
		m_tracerPts[0].y = m_pos_y;

		m_tracer_elapsed_ns = 0;
		}
		else
			m_tracer_elapsed_ns += elapsed_ns;
	}

	// update velocity based on acceleration
	m_velocity_y += (m_acceleration_y * elapsed_ns) / 1.0e9;

	// update position based on velocity
	double inc_x = (m_velocity_x * m_scale * elapsed_ns) / 1.0e9;
	double inc_y = (m_velocity_y * m_scale * elapsed_ns) / 1.0e9;
	//printf("X velocity: %.2f m/s, elapsed time: %.2f us, increment: %.5f\n", m_velocity_x, (double)elapsed_ns/1000, inc_x);
	m_pos_x += inc_x;
	m_pos_y += inc_y;
	//printf("X: %f Y: %f\n", m_pos_x, m_pos_y);
}

/*
void sim_object::accelerate_to_position(uint64_t dest_x, uint64_t dest_y)
{
}

void sim_object::accelerate_to_velocity(uint64_t accel_x, uint64_t accel_y, uint64_t vel_x, uint64_t vel_y)
{
	m_dest_velocity_x = vel_x;
	m_dest_velocity_y = vel_y;

	if (m_velocity_x < m_dest_velocity_x)
	{
	}
}
*/

bool CSimulation::Initialize(program_state *state, uint32_t w, uint32_t h)
{
	m_width = w;
	m_height = h;
	m_state = state;
	return true;
}

uint64_t CSimulation::UpdateSimulation(uint64_t abs_ns, uint64_t elapsed_ns)
{
	//printf("Updating model for the past %lu ns\n", elapsed_ns);

	// check for any events that have happened
	if (!sim_events.empty())
	{
		sim_event *event = sim_events.front();
		if (event->m_timestamp <= abs_ns)
		{
			event->m_cb(this, event->m_id, abs_ns);
			sim_events.pop_front();
		}
	}

	// update the sim items
	for (obj_list::iterator i = sim_objects.begin(); i != sim_objects.end(); i++)
	{
		(*i)->Update(elapsed_ns);
	}

	// check for new collisions
	CheckForCollision(abs_ns);

	return OK;
}

void CSimulation::Draw(SDL_Renderer* renderer)
{
//	printf("Starting to draw\n");

	// draw the background
	SDL_SetRenderDrawColor(renderer, 0x00, 0xB0, 0xE0, 0xFF);
	SDL_RenderClear(renderer);

	// draw the UI items
	for (obj_list::iterator i = sim_objects.begin(); i != sim_objects.end(); i++)
	{
		(*i)->Draw(renderer);
	}

	// draw annotations (collisions, projected paths, etc)
	if (m_collision && m_collision->draw)
	{
		filledCircleColor(renderer, m_collision->x, m_collision->y, 10, 0xFF0F0FE0);
		filledCircleColor(renderer, m_collision->x, m_collision->y, 5, 0xFF0F0FFF);
	}

}

bool CSimulation::CheckForCollision(uint64_t abs_ns)
{
	for (obj_list::iterator i = sim_objects.begin(); i != sim_objects.end(); i++)
	{
		for (obj_list::iterator j = i; j != sim_objects.end(); j++)
		{
			if (i != j)
			{
				sim_object *a = *i;
				sim_object *b = *j;
				printf("-- checking collision between %s and %s\n", a->name(), b->name());

	//printf("%s x: %f-%f\n", a->name(), a->x(), a->x() + a->width());
	//printf("%s x: %f-%f\n", b->name(), b->x(), b->x() + b->width());
	//printf("%s y: %f-%f\n", a->name(), a->y(), a->y() + a->height());
	//printf("%s y: %f-%f\n", b->name(), b->y(), b->y() + b->height());
				if ((a->x() > b->x() && a->x() < (b->x() + b->width()) ||
					(a->x() + a->width()) > b->x() && (a->x() + a->width()) < (b->x() + b->width())) &&
					(a->y() > b->y() && a->y() < (b->y() + b->height()) ||
					((a->y() + a->height()) > b->y() && (a->y() + a->height()) < (b->y() + b->height()))))
				{
					printf("%s collided with %s\n", (*i)->name(), (*j)->name());

					m_collision = new sim_collision;
					m_collision->a = (*i);
					m_collision->b = (*j);
					m_collision->x = (*i)->x() + (*i)->width()/2;
					m_collision->y = (*i)->y() + (*i)->height()/2;
					m_collision->timestamp = abs_ns;
					m_collision->draw = true;
					return COLLISION;
				}
			}
		}
	}

	return false;
}

bool CSimulation::who_collided(sim_collision *c, sim_object *a, sim_object *b)
{
	return (c->a == a && c->b == b || c->a == b && c->b == a);
}
