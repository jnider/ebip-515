#ifndef _MYSIM__H
#define  _MYSIM__H

#include "simulation.h"
#include "interaction.h"
#include "bip.h"

#define HZ_TO_NS(_hz)				(1000000000UL/_hz)
#define SECONDS_TO_NS(_n)			(1000000000UL * _n)
#define MS_TO_NS(_n)			      (1000000UL * _n)

#define GRAVITY 						9.81
#define SENSOR_FREQUENCY			60
#define MIN_BALL_VELOCITY			10
#define MAX_BALL_VELOCITY			35
#define ROBOT_VELOCITY				40
#define MAX_SENSOR_NOISE			0
#define CATCH_TOLERANCE				1.000

#define TIME_COLLISIONS_VISIBLE	SECONDS_TO_NS(1)
#define TIME_BEFORE_EGG				SECONDS_TO_NS(3)
#define TIME_BEFORE_BALL			SECONDS_TO_NS(2)

enum
{
	EVENT_DROP_EGG,
	EVENT_THROW_BALL
};

enum
{
	DIR_LEFT,
	DIR_RIGHT,
	DIR_STOP
};

class CMySimulation : public CSimulation
{
public:
	CMySimulation();
	~CMySimulation();

	bool Initialize(program_state *state, uint32_t w, uint32_t h);
	void Draw(SDL_Renderer* renderer);
	uint64_t UpdateSimulation(uint64_t abs_ns, uint64_t elapsed_ns);
	void DropEgg();
	void ThrowBall();
	void RobotMove(uint64_t direction);
	static void event_handler(CSimulation *s, uint64_t id, uint64_t timestamp);
	bool sensor_read_pos(sim_object *obj, uint64_t *x, uint64_t *y);
	void HandleEvent(SDL_Event *event);
	void OnCollision(uint64_t abs_ns, sim_object *a, sim_object *b);

protected:
	bool AddSensor(uint64_t index, sim_object *s);
	void UpdateCatchrateUI();
	void UpdateSensorUI(uint32_t i, uint64_t x_pos, uint64_t y_pos);

	int CreateInitialEnsemble();
	void UpdateEnsemble(uint64_t abs_ns, uint64_t elapsed_ns);

private:
	sim_object *ground;
	sim_object *robot;
	sim_object *bird;
	sim_object *egg;
	sim_object *player;
	sim_object *ball;
	TTF_Font* m_fontSans;
	sim_collision *m_collision;
	sim_object *m_sensors[MAX_SENSORS]; // dynamic array of sensors to read
	uint64_t m_num_sensors; // how many elements in the m_sensors array
	uint64_t m_sensor_elapsed; // time elapsed since the last sensor reading
	uint64_t m_sensor_delay; // how long to wait (ns) between sensor readings
	uint64_t m_catch;
	uint64_t m_trials;
	FILE *m_tracefile; // log of sensor readings in CSV format
	bool m_display_sensors; // should we display the sensor readings on-screen

	SDL_Surface* m_s_sensors[MAX_SENSORS]; // ui objects containing sensor text
	SDL_Surface* m_s_catchrate;
	SDL_Surface* m_s_training; // message if we are in training mode

	double m_sensorNoise[NUM_STATE_VARIABLES * NUM_STATE_VARIABLES];
	double m_est_state[NUM_STATE_VARIABLES];
	double m_predicted_state[NUM_STATE_VARIABLES];
	BIP *m_primitive;
	double *m_avg_trajectory;
};


#endif //  _MYSIM__H
