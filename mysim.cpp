#include "mysim.h"

CMySimulation::~CMySimulation()
{
	delete robot;
	delete bird;
	delete ball;
/*
	for (uint64_t i=0; i < m_num_sensors; i++)
		SDL_FreeSurface(m_s_sensors[i]);
*/
}

bool CMySimulation::Initialize(program_state *state, uint32_t w, uint32_t h)
{
	//printf("%s\n", __PRETTY_FUNCTION__);

	if (!CSimulation::Initialize(state, w, h))
		return false;

	m_fontSans = TTF_OpenFont("/usr/share/fonts/truetype/open-sans/OpenSans-Regular.ttf", 24);
	if (!m_fontSans)
	{
		printf("Can't find font Sans\n");
		return false;
	}

	ground = new sim_object(0, h-50);
	ground->set_width(w);
	ground->set_height(10);
	ground->set_name("ground");
	sim_objects.push_back(ground);

	robot = new sim_object(100, h-50-50);
	robot->set_width(50);
	robot->set_height(50);
	robot->set_name("robot");
	sim_objects.push_back(robot);

	player = new sim_object(25, ground->y() - 100);
	player->set_width(25);
	player->set_height(100);
	player->set_name("player");
	sim_objects.push_back(player);
	sim_events.push_back(new sim_event(TIME_BEFORE_EGG, EVENT_THROW_BALL, event_handler));
/*
	bird = new sim_object(0, 50);
	bird->set_width(20);
	bird->set_height(10);
	bird->set_velocity_x(25);
	bird->set_name("bird");
	sim_objects.push_back(bird);
*/

/*
	// after some time, tell the bird to drop an egg
	sim_events.push_back(new sim_event(TIME_BEFORE_EGG, event_handler));
*/

	// create the list of sensors to poll
	m_sensors[m_num_sensors++] = robot;
	printf("Capturing sensor data every %lu ns (%lu Hz)\n", m_sensor_delay, 1000000000/m_sensor_delay);
	return true;
}

void CMySimulation::DropEgg()
{
	if (egg)
	{
		printf("Egg already dropped!\n");
		return;
	}

	printf("Dropping egg @ %f x %f\n", bird->x(), bird->y());
	egg = new sim_object(bird->x(), bird->y() + bird->height());
	egg->set_width(20);
	egg->set_height(20);
	egg->set_velocity_x(bird->velocity_x());
	egg->set_acceleration_y(GRAVITY);
	egg->set_name("egg");
	egg->set_tracer_length(40);
	sim_objects.push_back(egg);
}

void CMySimulation::ThrowBall()
{
	if (ball)
	{
		printf("Ball already thrown!\n");
		return;
	}

	if (!player)
	{
		printf("No player!\n");
		return;
	}

	// calculate a random trajectory (separate x and y components)
	uint64_t x = rand() % (MAX_BALL_VELOCITY - MIN_BALL_VELOCITY);
	uint64_t y = rand() % (MAX_BALL_VELOCITY - MIN_BALL_VELOCITY);

	printf("Throwing ball x: %lu y: %lu\n", x, y);
	ball = new sim_object(player->x() + player->width(), player->y());
	ball->set_width(20);
	ball->set_height(20);
	ball->set_velocity_x(x + MIN_BALL_VELOCITY);
	ball->set_velocity_y((0 - (double)y - MIN_BALL_VELOCITY));
	ball->set_acceleration_y(GRAVITY);
	ball->set_name("ball");
	ball->set_tracer_length(40);
	sim_objects.push_back(ball);
	m_sensors[m_num_sensors++] = ball;
}

void CMySimulation::event_handler(CSimulation *s, uint64_t eventID, uint64_t timestamp)
{
	//printf("%lu: %s\n", timestamp, __PRETTY_FUNCTION__);

	CMySimulation *mysim = static_cast<CMySimulation*>(s);

	switch(eventID)
	{
	case EVENT_DROP_EGG:
		mysim->DropEgg();
		break;

	case EVENT_THROW_BALL:
		mysim->ThrowBall();
		break;
	}
}

uint64_t CMySimulation::UpdateSimulation(uint64_t abs_ns, uint64_t elapsed_ns)
{
	CSimulation::UpdateSimulation(abs_ns, elapsed_ns);

	// read the sensors
	if (m_sensor_elapsed > m_sensor_delay)
	{
		char message[256];

		m_sensor_elapsed -= m_sensor_delay;

		// read the set of sensors (degrees of freedom) for the controlled agent (robot) and observed agent(s) (person and ball)

		// prediction

		// movement

		// prepare the UI output by freeing the previous messages, and then filling in the new ones
		SDL_Color White = {0xFF, 0xF0, 0xF0};
		for (uint64_t i=0; i < m_num_sensors; i++)
		{
			uint64_t x_pos=0, y_pos=0;
			if (m_s_sensors[i])
			{
				SDL_FreeSurface(m_s_sensors[i]);
				m_s_sensors[i] = NULL;
			}

			sensor_read_pos(m_sensors[i], &x_pos, &y_pos);

			if (m_sensors[i])
			{
				snprintf(message, 256, "%s x:%lu y:%lu", m_sensors[i]->name(), x_pos, y_pos);
				m_s_sensors[i] = TTF_RenderText_Solid(m_fontSans, message, White);
			}
		}
	}
	else
	{
		m_sensor_elapsed += elapsed_ns;
	}

	{
		if (collision)
		{
			//if (strcmp(collision->a->name(), "egg") == 0)
			{
				collision->a->set_velocity_x(0);
				collision->a->set_velocity_y(0);
			}
		}
	}

	// remove old collisions
	if (collision && abs_ns - collision->timestamp > TIME_COLLISIONS_VISIBLE)
	{
		collision->draw = false;
		delete collision;
		collision = NULL;
	}

	return SIM_STATE_RUNNING;
}

void CMySimulation::RobotMove(uint64_t direction)
{
	switch (direction)
	{
	case DIR_RIGHT:
		robot->set_velocity_x(ROBOT_VELOCITY);
		break;

	case DIR_LEFT:
		robot->set_velocity_x(-ROBOT_VELOCITY);
		break;

	case DIR_STOP:
		robot->set_velocity_x(0);
		break;
	}
}

void CMySimulation::Draw(SDL_Renderer* renderer)
{
	CSimulation::Draw(renderer);

	if (display_sensors)
	{
		SDL_Rect Message_rect; //create a rect
		Message_rect.x = m_width - 220;  //controls the rect's x coordinate 
		Message_rect.y = m_height / 2; // controls the rect's y coordinte
		Message_rect.w = 220; // controls the width of the rect
		Message_rect.h = 50; // controls the height of the rect

		//Don't forget to free your surface and texture
		for (uint64_t i=0; i < m_num_sensors; i++)
		{
			Message_rect.y += Message_rect.h + 5;
			if (m_s_sensors[i])
			{
				SDL_Texture* txt_sensors = SDL_CreateTextureFromSurface(renderer, m_s_sensors[i]);
				SDL_RenderCopy(renderer, txt_sensors, NULL, &Message_rect);
				SDL_DestroyTexture(txt_sensors);
			}
		}
	}
}

bool CMySimulation::sensor_read_pos(sim_object *obj, uint64_t *x, uint64_t *y)
{
	int x_noise = 0, y_noise = 0;
	if (!obj)
		return false;

	if (MAX_SENSOR_NOISE)
	{
		// add some noise to the reading
		x_noise = (rand() % (MAX_SENSOR_NOISE ? MAX_SENSOR_NOISE : 1)) - (MAX_SENSOR_NOISE >> 1);
		y_noise = (rand() % (MAX_SENSOR_NOISE ? MAX_SENSOR_NOISE : 1)) - (MAX_SENSOR_NOISE >> 1);
	}

	*x = obj->x() + x_noise;
	*y = obj->y() + y_noise;
	return true;
}

void CMySimulation::HandleEvent(SDL_Event *event)
{
		switch(event->type)
		{
		case SDL_KEYDOWN:
			switch(event->key.keysym.sym)
			{
			case SDLK_ESCAPE:
				m_state->sim_running = SIM_STATE_STOPPED;
				m_state->quit = true;
				break;

			case SDLK_RIGHT:
				RobotMove(DIR_RIGHT);
				break;

			case SDLK_LEFT:
				RobotMove(DIR_LEFT);
				break;

			case SDLK_p:
				// toggle between running and paused
				if (m_state->sim_running == SIM_STATE_RUNNING)
					m_state->sim_running = SIM_STATE_PAUSED;
				else if (m_state->sim_running == SIM_STATE_PAUSED)
					m_state->sim_running = SIM_STATE_RUNNING;
				break;

			case SDLK_s:
				// toggle showing the sensor readings
				if (display_sensors)
					display_sensors = false;
				else
					display_sensors = true;
				break;

			case SDLK_r:
				printf("Restart simulation\n");
				m_state->sim_running = SIM_STATE_STOPPED;
				m_state->quit = false;
				break;

			default:
				printf("Got key %c\n", event->key.keysym.sym);
			}
			break;

		case SDL_KEYUP:
			switch(event->key.keysym.sym)
			{
			case SDLK_RIGHT:
			case SDLK_LEFT:
				RobotMove(DIR_STOP);
				break;
			}
		}
}

