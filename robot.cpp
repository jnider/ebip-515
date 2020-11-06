#include "robot.h"

void robot::Draw(SDL_Surface* s)
{
	m_rect.x = m_pos_x;
	m_rect.y = m_pos_y;
	printf("%s x=%u y=%u\n", __PRETTY_FUNCTION__, m_rect.x, m_rect.y); // __METHOD_NAME__
	SDL_FillRect(s, &m_rect, SDL_MapRGB(s->format, 0x1F, 0x02, 0x20));
}

void robot::Update(uint64_t nsec)
{
	//printf("%s\n", __PRETTY_FUNCTION__);
	// update position based on velocity
	m_pos_x += m_velocity_x;
	m_pos_y += m_velocity_y;
	//printf("X: %f Y: %f\n", m_pos_x, m_pos_y);
}

