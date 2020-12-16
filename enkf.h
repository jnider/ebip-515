#ifndef _ENKF__H
#define _ENKF__H

#include "bip.h"
#include "interaction.h"

class BIP;

class EnsembleKalmanFilter
{
public:
	EnsembleKalmanFilter(BIP *basis_model);
	~EnsembleKalmanFilter();
	void get_ensemble_covariance(double *covar);
	void get_projected_mean_covariance();
	void h(double *state);
	void localize(double *measurement, double *noise, double *phase, double *mean, double *covar);

protected:
	// mean values of all state variables (vector of NUM_STATE_VARIABLES) in the ensemble 
	void get_ensemble_mean(double *mean, double *ensemble);

private:
	BIP *m_basis_model;
	double phase; // current state of trial (in percent)
	double phase_velocity_ms; // how fast we are moving through the trial (percent per millisecond)
	double *m_ensemble; //D x E: the weights of each state variable in ensemble member
};

#endif // _ENKF__H
