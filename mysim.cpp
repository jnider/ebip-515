#include <dirent.h>
#include "mysim.h"
#include "interaction.h"

SDL_Color White = {0xFF, 0xF0, 0xF0};
SDL_Color Red = {0xFF, 0x00, 0x00};

CMySimulation::~CMySimulation()
{
	if (m_tracefile)
		fclose(m_tracefile);

	delete robot;
	delete bird;
	delete ball;
}

bool CMySimulation::Initialize(program_state *state, uint32_t w, uint32_t h)
{
	//DEBUG_PRINT("%s\n", __PRETTY_FUNCTION__);

	if (!CSimulation::Initialize(state, w, h))
		return false;

	m_fontSans = TTF_OpenFont("/usr/share/fonts/truetype/open-sans/OpenSans-Regular.ttf", 24);
	if (!m_fontSans)
	{
		DEBUG_PRINT("Can't find font Sans\n");
		return false;
	}

	ground = new sim_object(0, h-50, m_scale);
	ground->set_width(w);
	ground->set_height(50);
	ground->set_name("ground");
	sim_objects.push_back(ground);

	robot = new sim_object(100, h-50-50, m_scale);
	robot->set_width(50);
	robot->set_height(50);
	robot->set_name("robot");
	sim_objects.push_back(robot);

	player = new sim_object(25, ground->y() - 100, m_scale);
	player->set_width(25);
	player->set_height(100);
	player->set_name("player");
	sim_objects.push_back(player);
	sim_events.push_back(new sim_event(TIME_BEFORE_EGG, EVENT_THROW_BALL, event_handler));
/*
	bird = new sim_object(0, 50, m_scale);
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
	AddSensor(robot);
	DEBUG_PRINT("Capturing sensor data every %lu ns (%lu Hz)\n", m_sensor_delay, 1000000000/m_sensor_delay);

	if (state->training)
	{
		char *tmpstr=NULL;
		for(uint32_t counter = 0; counter < 1000; counter++)
		{
			if (!asprintf(&tmpstr, "%s/trace%03u.out", state->tracepath, counter))
				break;

			m_tracefile = fopen(tmpstr, "r");
			if (!m_tracefile)
				break;

			fclose(m_tracefile);
		}

		// open trace file for reading & writing (create or truncate)
		m_tracefile = fopen(tmpstr, "w+");

		if (!m_tracefile)
			printf("Error opening trace file %s\n", tmpstr);
		else
		{
			printf("Using trace file %s\n", tmpstr);
			fprintf(m_tracefile, "# version %u\n", VERSION);
		}
		free(tmpstr);
	}
	else
	{
		// Load previous traces to create an ensemble
		CreateInitialEnsemble();
	}

	UpdateCatchrateUI();

	if (m_state->training)
		m_s_training = TTF_RenderText_Solid(m_fontSans, "TRAINING", Red);

	return true;
}

void CMySimulation::DropEgg()
{
	if (egg)
	{
		DEBUG_PRINT("Egg already dropped!\n");
		return;
	}

	DEBUG_PRINT("Dropping egg @ %f x %f\n", bird->x(), bird->y());
	egg = new sim_object(bird->x(), bird->y() + bird->height(), m_scale);
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
		DEBUG_PRINT("Ball already thrown!\n");
		return;
	}

	if (!player)
	{
		DEBUG_PRINT("No player!\n");
		return;
	}

	// calculate a random trajectory (separate x and y components)
	uint64_t x = rand() % (MAX_BALL_VELOCITY - MIN_BALL_VELOCITY);
	uint64_t y = rand() % (MAX_BALL_VELOCITY - MIN_BALL_VELOCITY);

	DEBUG_PRINT("Throwing ball x: %lu y: %lu\n", x, y);
	ball = new sim_object(player->x() + player->width(), player->y(), m_scale);
	ball->set_width(20);
	ball->set_height(20);
	ball->set_velocity_x(x + MIN_BALL_VELOCITY);
	ball->set_velocity_y((0 - (double)y - MIN_BALL_VELOCITY));
	ball->set_acceleration_y(GRAVITY);
	ball->set_name("ball");
	ball->set_tracer_length(40);
	sim_objects.push_back(ball);

	AddSensor(ball);
}

bool CMySimulation::AddSensor(sim_object *s)
{
	if (m_num_sensors >= MAX_SENSORS)
	{
		DEBUG_PRINT("Can't add any more sensors\n");
		return false;
	}

	m_s_sensors[m_num_sensors] = NULL;
	m_sensors[m_num_sensors++] = s;
	return true;
}

void CMySimulation::event_handler(CSimulation *s, uint64_t eventID, uint64_t timestamp)
{
	DEBUG_PRINT("%lu: %s\n", timestamp, __PRETTY_FUNCTION__);

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
	uint64_t ret = CSimulation::UpdateSimulation(abs_ns, elapsed_ns);

	PropagateEnsemble(abs_ns, elapsed_ns);

	// read the sensors
	if (m_sensor_elapsed > m_sensor_delay)
	{
		m_sensor_elapsed -= m_sensor_delay;

		if (m_state->training && m_tracefile)
			fprintf(m_tracefile, "%010lu", abs_ns);

		// read all sensors
		for (uint64_t i=0; i < m_num_sensors; i++)
		{
			uint64_t x_pos=0, y_pos=0;
			// read the set of sensors for the controlled agent (robot) and observed agent(s) (person and ball)
			sensor_read_pos(m_sensors[i], &x_pos, &y_pos);

			if (m_state->training && m_tracefile)
				fprintf(m_tracefile, ",%lu,%lu", x_pos, y_pos);

			// update UI if necessary
			if (m_state->ui_visible)
				UpdateSensorUI(i, x_pos, y_pos);
		}

		if (m_state->training && m_tracefile)
			fputs("\n", m_tracefile);

		MeasurementUpdateEnsemble();

		// movement

	}
	else
	{
		m_sensor_elapsed += elapsed_ns;
	}


	// remove old collisions
	if (m_collision && abs_ns - m_collision->timestamp > TIME_COLLISIONS_VISIBLE)
	{
		m_collision->draw = false;
		delete m_collision;
		m_collision = NULL;
	}

	return 0;
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

	SDL_Rect Message_rect; //create a rect
	if (display_sensors)
	{
		Message_rect.x = m_width - 220;  //controls the rect's x coordinate 
		Message_rect.y = m_height / 2; // controls the rect's y coordinte
		Message_rect.w = 220; // controls the width of the rect
		Message_rect.h = 50; // controls the height of the rect

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

	// immediately below the sensors, write the catch rate
	if (m_s_catchrate)
	{
		Message_rect.y += Message_rect.h + 5;
		SDL_Texture* txt = SDL_CreateTextureFromSurface(renderer, m_s_catchrate);
		SDL_RenderCopy(renderer, txt, NULL, &Message_rect);
		SDL_DestroyTexture(txt);
	}
	
	// draw annotations (collisions, projected paths, etc)
	if (m_collision && m_collision->draw)
	{
		filledCircleColor(renderer, m_collision->x, m_collision->y, 10, 0xFF0F0FE0);
		filledCircleColor(renderer, m_collision->x, m_collision->y, 5, 0xFF0F0FFF);
	}

	if (m_state->training)
	{
		Message_rect.x = m_width / 2;  //controls the rect's x coordinate 
		Message_rect.y = m_height - 50; // controls the rect's y coordinte
		Message_rect.w = 100; // controls the width of the rect
		Message_rect.h = 40; // controls the height of the rect
		SDL_Texture* txt = SDL_CreateTextureFromSurface(renderer, m_s_training);
		SDL_RenderCopy(renderer, txt, NULL, &Message_rect);
		SDL_DestroyTexture(txt);
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
				if (m_state->training)
					RobotMove(DIR_RIGHT);
				break;

			case SDLK_LEFT:
				if (m_state->training)
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
				DEBUG_PRINT("Restart simulation\n");
				m_state->sim_running = SIM_STATE_STOPPED;
				m_state->quit = false;
				break;

			default:
				DEBUG_PRINT("Got key %c\n", event->key.keysym.sym);
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

void CMySimulation::OnCollision(uint64_t abs_ns, sim_object *a, sim_object *b)
{
	DEBUG_PRINT("Got collision\n");
	m_collision = new sim_collision;
	m_collision->a = a;
	m_collision->b = b;
	m_collision->x = a->x() + a->width()/2;
	m_collision->y = a->y() + a->height()/2;
	m_collision->timestamp = abs_ns;
	m_collision->draw = true;

	if (who_collided(m_collision, robot, ball))
	{
		DEBUG_PRINT("robot & ball collided\n");
		ball->set_velocity_x(0);
		ball->set_velocity_y(0);
		ball->set_acceleration_y(0);

		// if the ball landed on top of the robot, call it a 'catch'
		//DEBUG_PRINT("Bottom of ball: %f\n", ball->y() + ball->height());
		//DEBUG_PRINT("Top of robot: %f\n", robot->y());
		if (ball->y() + ball->height() - robot->y() < CATCH_TOLERANCE)
		{
			DEBUG_PRINT("catch! %s (%f)\n", m_collision->a->name(), ball->y() + ball->height() - robot->y());
			m_catch++;
			UpdateCatchrateUI();
		}
	}
	else if (who_collided(m_collision, ground, ball))
	{
		ball->set_velocity_x(0);
		ball->set_velocity_y(0);
		ball->set_acceleration_y(0);
	}
	m_state->sim_running = SIM_STATE_PAUSED;
}

void CMySimulation::UpdateCatchrateUI()
{
	char message[100];
	if (m_s_catchrate)
		SDL_FreeSurface(m_s_catchrate);
	snprintf(message, 100, "Trials:%lu Caught:%lu", m_state->trials, m_catch);
	m_s_catchrate = TTF_RenderText_Solid(m_fontSans, message, White);
}

void CMySimulation::UpdateSensorUI(uint32_t i, uint64_t x_pos, uint64_t y_pos)
{
	char message[100];
	if (m_s_sensors[i])
	{
		SDL_FreeSurface(m_s_sensors[i]);
		m_s_sensors[i] = NULL;
	}

	if (m_sensors[i])
	{
		snprintf(message, 100, "%s x:%lu y:%lu", m_sensors[i]->name(), x_pos, y_pos);
		m_s_sensors[i] = TTF_RenderText_Solid(m_fontSans, message, White);
	}
}

int CMySimulation::CreateInitialEnsemble()
{
	DIR *d;
	struct dirent *entry;
	char *tmpname;
	FILE *log;
	uint64_t num_traces = 0;
	uint64_t avg_length = 0;
	uint64_t total_length = 0;

	// load traces from the given directory
	printf("Loading from %s\n", m_state->tracepath);
	d = opendir(m_state->tracepath);
	if (d)
	{
		while ((entry = readdir(d)) != NULL)
		{
			if (strncmp(entry->d_name, "trace", 5) == 0)
			{
				if (!asprintf(&tmpname, "%s/%s", m_state->tracepath, entry->d_name))
					break;
				printf("%s\n", tmpname);
				log = fopen(tmpname, "r");
				CInteraction *interaction = new CInteraction(m_scale);
				interaction->Load(log);
				fclose(log);
				num_traces++;

				// determine the base function set (nonlinear least squares fitting?)

				// cheat by using kinematics (for now)
				//d = vi * t + (a* t^2)/2

				m_ensemble.push_back(interaction);
				//printf("Interaction length: %lu\n", interaction->Length());
				total_length += interaction->Length();
				m_mu.x += interaction->x();
				m_mu.y += interaction->y();
			}
		}
		closedir(d);
	}

	// calculate average interaction length
	avg_length = total_length / num_traces;
	printf("Average interaction length: %lu\n", avg_length);

	// calculate mu
	m_mu.x /= num_traces;
	m_mu.y /= num_traces;
	printf("Mu: x=%f y=%f\n", m_mu.x, m_mu.y);

	return 0;
}

void CMySimulation::PropagateEnsemble(uint64_t abs_ns, uint64_t elapsed_ns)
{
	for (ensemble_list::iterator i = m_ensemble.begin(); i != m_ensemble.end(); i++)
	{
		// matrix G is the 'constant velocity' model
		// We don't have a known velocity, but we can estimate the velocity by
		// calculating the difference
		(*i);
	}
}

void CMySimulation::MeasurementUpdateEnsemble()
{
	// transform the ensemble to the measurement space via the non-linear observation
	// function h(.) along with the deviation of each ensemble member from the sample mean:
	for (ensemble_list::iterator i = m_ensemble.begin(); i != m_ensemble.end(); i++)
	{
		//h(*i) - avg;
	}
}
