#include <dirent.h>
#include "mysim.h"
#include "interaction.h"

#define NUM_SAMPLES_TRAJECTORY 100
#define GROUND_HEIGHT 50
#define ROBOT_HEIGHT 50
#define PLAYER_HEIGHT 100
#define BALL_DIAMETER 20

SDL_Color White = {0xFF, 0xF0, 0xF0};
SDL_Color Red = {0xFF, 0x00, 0x00};

// Multiply an n x m matrix with an m x l matrix to produce an n x l matrix
static void MultiplyMatrix(double *A, double *B, double *C, int n, int m, int l)
{
   int i,j,k;
   for (i=0; i < n; i++)
   {
      for (j=0; j < l; j++)
      {
         for (k=0; k < m; k++)
			{
            C[i*n + j] +=  A[i*m + k] * B[k*n + j];
			}
      }
   }
}

static void DivideMatrix(double *A, double scalar, int n, int m)
{
   int i,j,k;

	if (scalar == 0)
		return;

   for (i=0; i < n; i++)
      for (j=0; j < m; j++)
			A[i*n + j] /= scalar;
}

/*
static void TransposeMatrix(double *A, double *B, int n, int m)
{
	for (int i=0; i < n; i++)
		for (int j=0; j < m; j++)
			B[j*n + i] = A[i*m + j];
}
*/

// must be square (n x n)
// Inversion function borrowed from:
// https://stackoverflow.com/questions/32043346/square-matrix-inversion-in-c
/*
static void InvertMatrix(double *A, double *B, int n)
{
	// set up matrix B with the identity matrix
	int i, j, k;
	double ratio, a;

	for (i=0; i < n; i++)
	{
		for (j=0; j < n; j++)
		{
			if (i == j)
				B[i*n + j] = 1;
			else
				B[i*n + j] = 0;
		}
	}

	float temp;
  for(k=0;k<n;k++)                                  //by some row operations,and the same row operations of
    {                                                       //Unit mat. I gives the inverse of matrix A
        temp=A[k*n + k];                   //'temp'  
        // stores the A[k][k] value so that A[k][k]  will not change
        for(j=0;j<n;j++)      //during the operation //A[i] //[j]/=A[k][k]  when i=j=k
        {
            A[k*n + j]/=temp;                                  //it performs // the following row operations to make A to unit matrix
            B[k*n + j]/=temp;                                  //R0=R0/A[0][0],similarly for I also R0=R0/A[0][0]
        }                                                   //R1=R1-R0*A[1][0] similarly for I

        for(i=0;i<n;i++)                              //R2=R2-R0*A[2][0]      ,,
        {
            temp=A[i*n + k];                       //R1=R1/A[1][1]
            for(j=0;j<n;j++)             //R0=R0-R1*A[0][1]
            {                                   //R2=R2-R1*A[2][1]
                if(i==k)
                    break;                      //R2=R2/A[2][2]
                A[i*n + j]-= A[k*n+j]*temp;          //R0=R0-R2*A[0][2]
                B[i*n + j]-= B[k*n+j]*temp;          //R1=R1-R2*A[1][2]
            }
        }
    }
}
*/

void print_matrix_double(double *A, int n, int m)
{
   int i,j,k;
   for (i=0; i < n; i++)
   {
      for (j=0; j < m; j++)
		{
			printf("%11.3f ", A[i *m + j]);
		}
		putc('\n', stdout);
	}
	putc('\n', stdout);
}

CMySimulation::CMySimulation() : robot(NULL), bird(NULL), egg(NULL), player(NULL), ball(NULL), m_fontSans(NULL), m_num_sensors(0),
		m_sensor_elapsed(0), m_sensor_delay(HZ_TO_NS(SENSOR_FREQUENCY)), m_collision(NULL), m_s_catchrate(NULL), m_catch(0),
		m_tracefile(NULL), m_s_training(NULL), m_avg_trajectory(NULL), m_display_sensors(false)
{
	// Initialize a BIP instance
	m_primitive = new BIP();
	for (int i=0; i < NUM_STATE_VARIABLES; i++)
	{
		m_est_state[i] = 0;
		m_predicted_state[i] = 0;
	}
	m_avg_trajectory = (double*)calloc(sizeof(double), NUM_STATE_VARIABLES * NUM_SAMPLES_TRAJECTORY);
}

CMySimulation::~CMySimulation()
{
	if (m_tracefile)
		fclose(m_tracefile);

	delete robot;
	delete bird;
	delete ball;
	delete m_primitive;
	free(m_avg_trajectory);
}

bool CMySimulation::Initialize(program_state *state, uint32_t w, uint32_t h)
{
	DEBUG_PRINT("%s\n", __PRETTY_FUNCTION__);

	if (!CSimulation::Initialize(state, w, h))
		return false;

	// set number of threads
	//openblas_set_num_threads(2);

	m_fontSans = TTF_OpenFont("/usr/share/fonts/truetype/open-sans/OpenSans-Regular.ttf", 24);
	if (!m_fontSans)
	{
		DEBUG_PRINT("Can't find font Sans\n");
		return false;
	}

	ground = new sim_object(w/2, h-(GROUND_HEIGHT/2), m_scale);
	ground->set_width(w);
	ground->set_height(GROUND_HEIGHT);
	ground->set_name("ground");
	sim_objects.push_back(ground);

	robot = new sim_object(100, ground->y()-(ground->height()/2)-(ROBOT_HEIGHT/2), m_scale);
	robot->set_width(50);
	robot->set_height(ROBOT_HEIGHT);
	robot->set_name("robot");
	sim_objects.push_back(robot);

	player = new sim_object(25/2, ground->y()-(ground->height()/2)-(PLAYER_HEIGHT/2), m_scale);
	player->set_width(25);
	player->set_height(PLAYER_HEIGHT);
	player->set_name("player");
	sim_objects.push_back(player);
	sim_events.push_back(new sim_event(TIME_BEFORE_BALL, EVENT_THROW_BALL, event_handler));

	ball = new sim_object(player->x() + player->width()/2 + BALL_DIAMETER/2 + 5, robot->y() - robot->height()/2 - BALL_DIAMETER/2 - 10, m_scale);
	ball->set_width(BALL_DIAMETER);
	ball->set_height(BALL_DIAMETER);
	ball->set_name("ball");
	ball->set_tracer_length(40);
	sim_objects.push_back(ball);

	AddSensor(SENSOR_BALL, ball);
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
	AddSensor(SENSOR_PLAYER, player);
	AddSensor(SENSOR_ROBOT, robot);
	DEBUG_PRINT("Capturing sensor data every %f ms (%lu Hz)\n", m_sensor_delay/(double)1000000, 1000000000/m_sensor_delay);

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
			fprintf(m_tracefile, "# timestamp,player,robot,ball\n");
		}
		free(tmpstr);
	}
	else
	{
		// Load previous traces to create an ensemble
		CreateInitialEnsemble();

		// Compute the phase mean and phase velocities from the demonstrations.
		double phase_velocity_mean;
		double phase_velocity_var;
		m_primitive->get_phase_stats(&phase_velocity_mean, &phase_velocity_var);
		printf("Phase velocity mean: %f %%/sample  Variance: %f\n", phase_velocity_mean*100, phase_velocity_var);

		m_primitive->create_initial_ensemble();

		m_primitive->get_mean_trajectory(0, 1, NUM_SAMPLES_TRAJECTORY, m_avg_trajectory);
		printf("Avg. trajectory:\n");
		print_matrix_double(m_avg_trajectory, NUM_STATE_VARIABLES, NUM_SAMPLES_TRAJECTORY);
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
/*
	if (ball)
	{
		DEBUG_PRINT("Ball already thrown!\n");
		return;
	}
*/

	if (!player)
	{
		DEBUG_PRINT("No player!\n");
		return;
	}

	// calculate a random trajectory (separate x and y components)
	uint64_t x = rand() % (MAX_BALL_VELOCITY - MIN_BALL_VELOCITY);
	uint64_t y = rand() % (MAX_BALL_VELOCITY - MIN_BALL_VELOCITY);

	DEBUG_PRINT("Throwing ball x: %lu y: %lu\n", x, y);
	ball->set_velocity_x(x + MIN_BALL_VELOCITY);
	ball->set_velocity_y((0 - (double)y - MIN_BALL_VELOCITY));
	ball->set_acceleration_y(GRAVITY);
}

bool CMySimulation::AddSensor(uint64_t index, sim_object *s)
{
	if (m_num_sensors >= MAX_SENSORS)
	{
		DEBUG_PRINT("Can't add any more sensors\n");
		return false;
	}

	m_s_sensors[index] = NULL;
	m_sensors[index] = s;
	m_num_sensors++;

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
	//uint64_t x_pos[MAX_SENSORS];
	//uint64_t y_pos[MAX_SENSORS];

	uint64_t ret = CSimulation::UpdateSimulation(abs_ns, elapsed_ns);

	if (m_sensor_elapsed > m_sensor_delay)
	{
		m_sensor_elapsed -= m_sensor_delay;

		if (m_state->training && m_tracefile)
			fprintf(m_tracefile, "%010lu", abs_ns);

		// read the sensors
		for (uint64_t i=0; i < m_num_sensors; i++)
		{
			if (m_state->training && m_tracefile)
				fprintf(m_tracefile, ",%lu,%lu", (uint64_t)m_sensors[i]->x(), (uint64_t)m_sensors[i]->y());

			// update UI if necessary
			if (m_state->ui_visible)
				UpdateSensorUI(i, m_sensors[i]->x(), m_sensors[i]->y());
		}

		if (m_state->training && m_tracefile)
			fputs("\n", m_tracefile);

		// update the model
		if (!m_state->training)
			UpdateEnsemble(abs_ns, elapsed_ns);

		// movement
		if (!m_state->training)
		{
			printf("Robot predicted=%f actual=%f\n", m_est_state[STATE_VAR_ROBOT_X], m_sensors[SENSOR_ROBOT]->x());
			if (m_est_state[STATE_VAR_ROBOT_X] > m_sensors[SENSOR_ROBOT]->x())
			{
				RobotMove(DIR_RIGHT);
			}

			if (m_est_state[STATE_VAR_ROBOT_X] < m_sensors[SENSOR_ROBOT]->x())
			{
				RobotMove(DIR_LEFT);
			}

			// don't let the robot run into the player
			//printf("Robot vel: %f x:%f player x: %f player width: %f\n", robot->velocity_x(), robot->x(), player->x(), player->width());
			if ((robot->velocity_x() < 1) && robot->x() < (player->x() + player->width() + 25))
			{
				RobotMove(DIR_STOP);
			}
		}
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
	//printf("Robot moving %lu\n", direction);
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
	SDL_Rect Message_rect; //create a rect

	CSimulation::Draw(renderer);

	// draw the average trajectory
	if (!m_state->training && m_avg_trajectory)
	{
		for (unsigned int index = 0; index < NUM_SAMPLES_TRAJECTORY; index++)
		{
			filledCircleColor(renderer,
				m_avg_trajectory[NUM_SAMPLES_TRAJECTORY * STATE_VAR_BALL_X + index],
				m_avg_trajectory[NUM_SAMPLES_TRAJECTORY * STATE_VAR_BALL_Y + index],
				4, 0xF010A010);
		}
	}

	// draw predicted location of ball
	if (!m_state->training)
	{
		unsigned int color = 0x0000FFFF | 0xFF000000;
		filledCircleColor(renderer,
			m_predicted_state[STATE_VAR_BALL_X],
			m_predicted_state[STATE_VAR_BALL_Y],
			5, color);
	}

	if (m_display_sensors)
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
	{
		*x = 0;
		*y = 0;
		return false;
	}

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
				if (m_display_sensors)
					m_display_sensors = false;
				else
					m_display_sensors = true;
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
	m_collision->x = (a->x() + b->x()) / 2;
	m_collision->y = (a->y() + b->y()) / 2;
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
		if (ball->y() + ball->height()/2 - robot->y()/2 < CATCH_TOLERANCE)
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

	printf("%s\n", __func__);

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
				//printf("%s: ", tmpname);
				log = fopen(tmpname, "r");
				CInteraction *interaction = new CInteraction(m_scale);
				interaction->Load(log);
				fclose(log);
				m_primitive->add_demonstration(interaction);
				num_traces++;

				// for now, stop reading after we have enough members. Later, update this to sample members at random
				if (num_traces == NUM_ENSEMBLE_MEMBERS)
					break;
			}
		}
		closedir(d);
	}

	if (num_traces < NUM_ENSEMBLE_MEMBERS)
	{
		printf("ERROR: not enough trials for %u ensemble members\n", NUM_ENSEMBLE_MEMBERS);
		return -1;
	}

	return 0;
}

// Motion update step
// Rather than multiplying by the A matrix, just read the next set of values at this phase
void CMySimulation::UpdateEnsemble(uint64_t abs_ns, uint64_t elapsed_ns)
{
	//double *mean = (double *)calloc(sizeof(double), NUM_STATE_VARIABLES);

	//printf("$ PHASE : %f %f\n", m_weights[STATE_VAR_PHASE], m_weights[STATE_VAR_PHASE_VEL]);
	printf("$ BALL : %f %f\n", m_est_state[STATE_VAR_BALL_X], m_est_state[STATE_VAR_BALL_Y]);
	printf("$ ROBOT : %f\n", m_est_state[STATE_VAR_ROBOT_X]);

	printf("Propagating at time %f ms\n", (double)abs_ns/(double)1000000);
/*
	double *gen_trajectory = m_primitive->generate_probable_trajectory_recursive(trajectory, observation_noise, active_dofs, num_samples,
		1, &phase, mean, &var);

	free(gen_trajectory);
*/

	//m_primitive->get_ensemble_at(next_phase, nextState);

	// read the set of sensors for the controlled agent (robot) and observed agent(s) (person and ball)
	uint64_t x_pos, y_pos;
	double sensors[NUM_STATE_VARIABLES];

	sensor_read_pos(m_sensors[SENSOR_ROBOT], &x_pos, &y_pos);
	sensors[STATE_VAR_ROBOT_X] = x_pos;
	//sensors[STATE_VAR_ROBOT_Y] = y_pos;
	sensor_read_pos(m_sensors[SENSOR_BALL], &x_pos, &y_pos);
	sensors[STATE_VAR_BALL_X] = x_pos;
	sensors[STATE_VAR_BALL_Y] = y_pos;

	// set random noise
/*
	int i,j;
	double noise_range = 0.00001;
	for (i=0; i < NUM_STATE_VARIABLES; i++)
	{
		for (j=0; j < NUM_STATE_VARIABLES; j++)
		{
			m_sensorNoise[i*NUM_STATE_VARIABLES + j] = ((double)rand()/(double)RAND_MAX * noise_range) - (noise_range/2);
		}
	}
*/
	double sample = ((double)abs_ns / 1000000000) * (double)SENSOR_FREQUENCY;
	printf("sample: %f\n", sample);
	m_primitive->estimate_state(sample, sensors, NULL, m_est_state);
	//m_primitive->get_mean_trajectory(0, 1, NUM_SAMPLES_TRAJECTORY, m_avg_trajectory);


	//printf("^ PHASE : %f %f\n", m_est_state[STATE_VAR_PHASE], m_est_state[STATE_VAR_PHASE_VEL]);
	//printf("^ PHASE : %f\n", m_est_state[STATE_VAR_PHASE]);
	printf("^ BALL : %f %f\n", m_est_state[STATE_VAR_BALL_X], m_est_state[STATE_VAR_BALL_Y]);
	printf("^ ROBOT : %f\n", m_est_state[STATE_VAR_ROBOT_X]);
	//printf("> PHASE : %f %f\n", m_predicted_state[STATE_VAR_PHASE], m_predicted_state[STATE_VAR_PHASE_VEL]);
	//printf("> PHASE : %f\n", m_predicted_state[STATE_VAR_PHASE]);
	//printf("> BALL : %f %f\n", m_predicted_state[STATE_VAR_BALL_X], m_predicted_state[STATE_VAR_BALL_Y]);
	//printf("> ROBOT : %f\n", m_predicted_state[STATE_VAR_ROBOT_X]);
}

