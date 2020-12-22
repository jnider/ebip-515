#ifndef _BIP__H
#define _BIP__H

#include <list>
#include "interaction.h"

enum
{
	ENSEMBLE_STATE_PHASE,
	ENSEMBLE_STATE_PHASE_VEL,
	ENSEMBLE_STATE_WEIGHT,
	NUM_ENSEMBLE_STATES
};

enum
{
	SENSOR_PLAYER,
	SENSOR_ROBOT,
	SENSOR_BALL,
	MAX_SENSORS
};

typedef std::list<CInteraction*> interaction_list;

class EnsembleKalmanFilter;

class BIP
{
public:
	BIP();
	~BIP();
	void add_demonstration(CInteraction *interaction);
	void get_phase_stats(double *phase_velocity_mean, double *phase_velocity_var);
	void get_mean_trajectory(double range_start, double range_end, unsigned int num_samples, double *trajectory);

	void get_ensemble_mean(double *mean, double *ensemble);
	void create_initial_ensemble();
	void estimate_state(double phase, double *sensors, double *sensorNoise, double *predictedState);
	void get_weighted_mean(double *matrix);
	

protected:
	void generate_noise(double *matrix, double range, int n, int m);
	void hx(double *matrix);
	void get_ha_matrix(double *hx, double *ha);
	void propagate_ensemble(double sample);
	void add_sensor_noise(double *sensors, double *obs, double range, int m, int n);
	void apply_weights(int member, double *sample);

private:
	interaction_list m_interactions;
	double m_weights[NUM_ENSEMBLE_STATES * NUM_ENSEMBLE_MEMBERS]; // weights representing each ensemble member - rename as m_ensemble (B x E)
};

#endif // _BIP__H
