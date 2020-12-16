#ifndef _BIP__H
#define _BIP__H

#include <list>
#include "interaction.h"
#include "enkf.h"

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
	void export_data(const char *filename);
	void import_data(const char *filename);
	void compute_standardization(double *trajectory);
	void add_demonstration(CInteraction *interaction);
	double* get_mean_trajectory(unsigned int num_samples);
	void get_approximate_trajectory();
	void get_approximate_trajectory_derivative();
	void get_probability_distribution();
	void get_phase_stats(double *phase_velocity_mean, double *phase_velocity_var);
	void basis_transform();
	double* basis_inverse_transform(double range_start, double range_end, unsigned int num_samples);
	//void get_basis_weight_parameters(double *mean, double *var);
	double* generate_probable_trajectory_recursive(double *trajectory, double *observation_noise, double *active_dofs, unsigned int num_samples,
		double starting_phase, double *phase, double *mean, double *var);
	//void set_filter(EnsembleKalmanFilter *f) { m_filter = f; }
	void apply_coefficients(double phase, double *weights, double *trajectory);

	void get_ensemble_at(double phase, double *ensemble);
	void measurement_update(double phase, double *sensors, double *sensorNoise);
	void predict_outcome(double phase, double *weights, double *predictedState);
	void get_ensemble_mean(double *mean, double *ensemble);
	

protected:
	void init_scalers();
	void estimate_state(double phase, double *ensemble, double *predictedState);

private:
	double m_basis_weights[NUM_STATE_VARIABLES];
	interaction_list m_interactions;
	double m_ensemble[NUM_ENSEMBLE_MEMBERS * NUM_STATE_VARIABLES];
	double *KalmanGain;
};

#endif // _BIP__H
