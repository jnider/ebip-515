#include "bip.h"
#include <cblas.h>
#include <lapacke.h>

extern void print_matrix_double(double *A, int n, int m);

// subtract a row vector B (size m) from A n x m matrix, put result in C (n x m matrix)
static void Matrix_Subtract_Vector(double *A, double *B, double *C, int n, int m)
{
   int i,j;
   for (i=0; i < n; i++)
   {
      for (j=0; j < m; j++)
      {
			C[i*m + j] = A[i*m + j] - B[i];
		}
	}
}

BIP::BIP()
{
	init_scalers();
	KalmanGain = (double *)calloc(sizeof(double), NUM_ENSEMBLE_MEMBERS * NUM_ENSEMBLE_MEMBERS);

	for (int i=0; i < NUM_ENSEMBLE_MEMBERS; i++)
	{
		for (int j=0; j < NUM_STATE_VARIABLES; j++)
		{
			m_ensemble[i * NUM_STATE_VARIABLES + j] = 0;
		}
	}
}

BIP::~BIP()
{
	free(KalmanGain);
}

void BIP::export_data(const char *filename)
{
}

void BIP::import_data(const char *filename)
{
}

void BIP::compute_standardization(double *trajectory)
{
}

void BIP::add_demonstration(CInteraction *interaction)
{
	m_interactions.push_back(interaction);
	//m_basis_weights.push_back(1);
}

/* returns matrix D x num_samples */
double* BIP::get_mean_trajectory(unsigned int num_samples)
{
	//double mean[MAX_LATENT_FUNCTIONS];
	//double var[MAX_LATENT_FUNCTIONS * MAX_LATENT_FUNCTIONS];

	printf("%s num_samples=%u\n", __func__, num_samples);
	//get_basis_weight_parameters(mean, var);
	return basis_inverse_transform(0, 1, num_samples);
}

void BIP::get_approximate_trajectory()
{
}

void BIP::get_approximate_trajectory_derivative()
{
}

/* for debugging/visualization only */
void BIP::get_probability_distribution()
{
}

void BIP::get_phase_stats(double *phase_velocity_mean, double *phase_velocity_var)
{
	double tmp = 0;

	for (interaction_list::iterator i = m_interactions.begin();
		i != m_interactions.end(); i++)
	{
		tmp += (double)(*i)->length();
	}

	*phase_velocity_mean = (double)1 / (double)(tmp / (double)m_interactions.size());

	//The variance is the average of the squared deviations from the mean, i.e., var = mean(abs(x - x.mean())**2).
	double diff;
	tmp = 0;
	for (interaction_list::iterator i = m_interactions.begin();
		i != m_interactions.end(); i++)
	{
		diff = ((double)1/(double)(*i)->length()) - *phase_velocity_mean;
		tmp += (diff * diff);
	}
	
	*phase_velocity_var = tmp / m_interactions.size();
}

void BIP::basis_transform()
{
}

// ensemble: D x E
void BIP::get_ensemble_at(double phase, double *ensemble)
{
	unsigned int demonstration = 0;
	double sample[NUM_STATE_VARIABLES];

	// look up the sample at this phase directly from the demonstrations
	for (interaction_list::iterator i = m_interactions.begin(); i != m_interactions.end(); i++)
	{
		(*i)->get_sample(phase, sample);

		ensemble[NUM_ENSEMBLE_MEMBERS * STATE_VAR_PHASE + demonstration] = sample[STATE_VAR_PHASE];
		ensemble[NUM_ENSEMBLE_MEMBERS * STATE_VAR_PHASE_VEL + demonstration] = sample[STATE_VAR_PHASE_VEL];
		ensemble[NUM_ENSEMBLE_MEMBERS * STATE_VAR_BALL_X + demonstration] = sample[STATE_VAR_BALL_X];
		ensemble[NUM_ENSEMBLE_MEMBERS * STATE_VAR_BALL_Y + demonstration] = sample[STATE_VAR_BALL_Y];
		ensemble[NUM_ENSEMBLE_MEMBERS * STATE_VAR_ROBOT_X + demonstration] = sample[STATE_VAR_ROBOT_X];
		demonstration++;
	}
}

/* Transforms the given basis space weights to measurement space for the given phase values
	x Vector of dimension T containing the phase values that the basis space weights should be projected at.
*/
double* BIP::basis_inverse_transform(double range_start, double range_end, unsigned int num_samples)
{
	double phase = range_start;
	double stepping = (range_end - range_start) / (double)num_samples;
	double *trajectory = (double*)calloc(sizeof(double), NUM_STATE_VARIABLES * num_samples);
	double sample[NUM_STATE_VARIABLES];

	printf("%s stepping=%f\n", __func__, stepping);

	for (unsigned int sample_index = 0; sample_index < num_samples; sample_index++)
	{
		unsigned int valid_samples = 0;
		phase += stepping;
		trajectory[sample_index + num_samples * STATE_VAR_BALL_X] = 0;
		trajectory[sample_index + num_samples * STATE_VAR_BALL_Y] = 0;
		trajectory[sample_index + num_samples * STATE_VAR_ROBOT_X] = 0;

		// look up the sample at this phase directly from the demonstrations
		for (interaction_list::iterator i = m_interactions.begin(); i != m_interactions.end(); i++)
		{
			(*i)->get_sample(phase, sample);
			//printf("Got sample at phase %f:\n", phase);
			//print_matrix_double(sample, 1, NUM_STATE_VARIABLES);

			// apply the sample to the trajectory (requires a transpose)
			if (sample[STATE_VAR_BALL_X] && sample[STATE_VAR_BALL_Y])
			{
				trajectory[sample_index + num_samples * STATE_VAR_BALL_X] += sample[STATE_VAR_BALL_X];
				trajectory[sample_index + num_samples * STATE_VAR_BALL_Y] += sample[STATE_VAR_BALL_Y];
				trajectory[sample_index + num_samples * STATE_VAR_ROBOT_X] += sample[STATE_VAR_ROBOT_X];
				valid_samples++;
			}
		}
		// calculate the average
		trajectory[sample_index + num_samples * STATE_VAR_BALL_X] /= valid_samples;
		trajectory[sample_index + num_samples * STATE_VAR_BALL_Y] /= valid_samples;
		trajectory[sample_index + num_samples * STATE_VAR_ROBOT_X] /= valid_samples;
	}
	return trajectory;
}

/*
	Gets the mean and covariance for the trained demonstrations.

   @return mean Vector of dimension D containing the sample mean of the trained basis weights.
   @return var Matrix of dimension D x D containing the sample covariance of the trained basis weights.
*/
/*
void BIP::get_basis_weight_parameters(double *mean, double *covariance)
{
	unsigned int index = 0;

	printf("%s: %lu demonstrations\n", __func__, m_basis_weights.size());
	for (index = 0; index < NUM_STATE_VARIABLES; index++)
		mean[index] = 0;

	
	for (interaction_list::iterator i = m_interactions.begin(); i != m_interactions.end(); i++)
	{
		for (index = 0; index < NUM_STATE_VARIABLES; index++)
		{
			mean[index] += ;
		(*i)->nonlinear_fn(phase, sample);
		}
	}

	for (index = 0; index < MAX_LATENT_FUNCTIONS; index++)
		mean[index] /= NUM_ENSEMBLE_MEMBERS;

}
*/

/*
Performs inference over the given trajectory and returns the most probable future trajectory
*/
double* BIP::generate_probable_trajectory_recursive(double *trajectory, double *observation_noise, double *active_dofs, unsigned int num_samples,
	double starting_phase, double *phase, double *mean, double *var)
{
	double target_phase;

	printf("%s\n", __func__);
	//m_filter->localize(trajectory, observation_noise, phase, mean, var);

	target_phase = starting_phase;

	// clip the target_phase to keep it in range
	if (target_phase > 1.0)
		target_phase = 1.0;
	if (target_phase < 0.0)
		target_phase = 0.0;

	// Create a sequence from the stored basis weights
	return basis_inverse_transform(target_phase, 1, num_samples);
}

// initialize scaling groups
void BIP::init_scalers()
{
	// for now, do nothing
	// ne
}

/*
   Applies the given weights to this basis model. Projects a basis state to the measurement space.
	weights: vector (D) of weights
	state: vector (D) of weighted sample
*/
void BIP::apply_coefficients(double phase, double *weights, double *state)
{
	printf("%s\n", __func__);

	
	if (!weights)
		printf("Bad weights\n");
	if (!state)
		printf("Bad state\n");

	//printf("Got weights:\n");
	//print_matrix_double(weights, 1, NUM_STATE_VARIABLES);
	
	// look up the sample at this phase directly from the demonstrations
	double sample[NUM_STATE_VARIABLES];
	for (interaction_list::iterator i = m_interactions.begin(); i != m_interactions.end(); i++)
	{
		(*i)->get_sample(phase, sample);

		//memcpy(state, sample
	}
}

void BIP::estimate_state(double nextPhase, double *ensemble, double *predictedState)
{
	double *ha = (double *)calloc(sizeof(double), NUM_STATE_VARIABLES * NUM_ENSEMBLE_MEMBERS);// HtAt
	double *mean = (double *)calloc(sizeof(double), NUM_STATE_VARIABLES);// vector of averages used to derive At
	double *S = (double *)calloc(sizeof(double), NUM_STATE_VARIABLES * NUM_STATE_VARIABLES);		// Innovation co-variance
	double *Sinv = (double *)calloc(sizeof(double), NUM_ENSEMBLE_MEMBERS * NUM_ENSEMBLE_MEMBERS);
	double *A_matrix = (double *)calloc(sizeof(double), NUM_STATE_VARIABLES * NUM_ENSEMBLE_MEMBERS);
	double *partialKalman = (double *)calloc(sizeof(double), NUM_STATE_VARIABLES * NUM_STATE_VARIABLES);
	double *nextEnsemble = (double *)calloc(sizeof(double), NUM_STATE_VARIABLES * NUM_ENSEMBLE_MEMBERS);// HtXt|t-1
	unsigned int index;

		// a is the deviation of the current state from the mean
		get_ensemble_mean(mean, ensemble);
		Matrix_Subtract_Vector(ensemble, mean, A_matrix, NUM_STATE_VARIABLES, NUM_ENSEMBLE_MEMBERS);
		printf("A:\n");
		print_matrix_double(ha, NUM_STATE_VARIABLES, NUM_ENSEMBLE_MEMBERS);

		// hx matrix
		get_ensemble_at(nextPhase, nextEnsemble);
		//printf("next ensemble @ %f:\n", nextPhase);
		//print_matrix_double(nextEnsemble, NUM_STATE_VARIABLES, NUM_ENSEMBLE_MEMBERS);
		
		get_ensemble_mean(predictedState, nextEnsemble);
		//printf("next mean:\n");
		//print_matrix_double(predictedState, NUM_STATE_VARIABLES, 1);

		// ha is the deviation of the predicted state from the mean
		Matrix_Subtract_Vector(nextEnsemble, predictedState, ha, NUM_STATE_VARIABLES, NUM_ENSEMBLE_MEMBERS);
		printf("ha:\n");
		print_matrix_double(ha, NUM_STATE_VARIABLES, NUM_ENSEMBLE_MEMBERS);

		// S is the innovation covariance
		cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasTrans,
			NUM_STATE_VARIABLES, NUM_STATE_VARIABLES, NUM_ENSEMBLE_MEMBERS, (double)1/(double)(NUM_ENSEMBLE_MEMBERS-1),
			ha, NUM_ENSEMBLE_MEMBERS,
			ha, NUM_ENSEMBLE_MEMBERS, 0, S, NUM_STATE_VARIABLES);
		// TODO add some noise
		//Matrix_Add(ha, R);
		printf("S:\n");
		print_matrix_double(S, NUM_STATE_VARIABLES, NUM_STATE_VARIABLES);

		// Calculate S-inverse
		int pivots[NUM_STATE_VARIABLES];
		LAPACKE_dgetrf(LAPACK_ROW_MAJOR, NUM_STATE_VARIABLES, NUM_STATE_VARIABLES, S, NUM_STATE_VARIABLES, pivots);
		LAPACKE_dgetri(LAPACK_ROW_MAJOR, NUM_STATE_VARIABLES, S, NUM_STATE_VARIABLES, pivots);
		printf("Sinv:\n");
		print_matrix_double(S, NUM_STATE_VARIABLES, NUM_STATE_VARIABLES);

		// Calculate Kalman gain
		cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasTrans,
			NUM_STATE_VARIABLES, NUM_STATE_VARIABLES, NUM_ENSEMBLE_MEMBERS, 1,
			A_matrix, NUM_ENSEMBLE_MEMBERS,
			ha, NUM_ENSEMBLE_MEMBERS, 0, partialKalman, NUM_STATE_VARIABLES);
		printf("partial K:\n");
		print_matrix_double(partialKalman, NUM_STATE_VARIABLES, NUM_STATE_VARIABLES);
		cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans,
			NUM_STATE_VARIABLES, NUM_STATE_VARIABLES, NUM_STATE_VARIABLES, (double)1/(double)(NUM_ENSEMBLE_MEMBERS-1),
			partialKalman, NUM_STATE_VARIABLES,
			Sinv, NUM_STATE_VARIABLES, 0, KalmanGain, NUM_STATE_VARIABLES);
		printf("K:\n");
		print_matrix_double(KalmanGain, NUM_STATE_VARIABLES, NUM_STATE_VARIABLES);

		cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans,
			NUM_STATE_VARIABLES, NUM_STATE_VARIABLES, NUM_STATE_VARIABLES, (double)1/(double)(NUM_ENSEMBLE_MEMBERS-1),
			KalmanGain, NUM_STATE_VARIABLES,
			predictedState, NUM_STATE_VARIABLES, 1, ensemble, NUM_STATE_VARIABLES);

	free(ha);
	free(mean);
	free(S);
	free(Sinv);
	free(A_matrix);
	free(partialKalman);
	free(nextEnsemble);
}

void BIP::predict_outcome(double nextPhase, double *weights, double *predictedState)
{
	double *nextEnsemble = (double *)calloc(sizeof(double), NUM_STATE_VARIABLES * NUM_ENSEMBLE_MEMBERS);
	//memcpy(nextEnsemble, m_ensemble, sizeof(double) * NUM_STATE_VARIABLES * NUM_ENSEMBLE_MEMBERS);
	get_ensemble_at(nextPhase, nextEnsemble);
	while (nextPhase < 1)
	{
		estimate_state(nextPhase, nextEnsemble, predictedState);

		nextPhase += 0.01;
	}
	get_ensemble_mean(predictedState, nextEnsemble);
	free(nextEnsemble);
}

void BIP::measurement_update(double phase, double *sensors, double *sensorNoise)
{
	double predictedState[NUM_STATE_VARIABLES];
	estimate_state(phase, m_ensemble, predictedState);
}

/*
	state ensemble D x E
	mean Vector of dimension D
*/
void BIP::get_ensemble_mean(double *mean, double *ensemble)
{
	unsigned int index;
	unsigned int vars;

	for (unsigned int vars = 0; vars < NUM_STATE_VARIABLES; vars++)
		mean[vars] = 0;

	if (!ensemble)
		printf("No ensemble\n");

	for (vars = 0; vars < NUM_STATE_VARIABLES; vars++)
	{
		for (index=0; index < NUM_ENSEMBLE_MEMBERS; index++)
		{
			mean[vars] += ensemble[NUM_ENSEMBLE_MEMBERS * vars + index];
		}
		mean[vars] /= NUM_ENSEMBLE_MEMBERS;
	}
}

