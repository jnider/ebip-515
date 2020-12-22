#include "bip.h"
#include <cblas.h>
#include <lapacke.h>

extern void print_matrix_double(double *A, int n, int m);

// subtract a vector B (size m) from A n x m matrix, put result in C (n x m matrix)
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

static void Matrix_Add_Matrix(double *A, double *B, double *C, int n, int m)
{
   int i,j;
   for (i=0; i < n; i++)
   {
      for (j=0; j < m; j++)
      {
			C[i*m + j] = A[i*m + j] + B[i*m + j];
		}
	}
}

static void Matrix_Subtract_Matrix(double *A, double *B, double *C, int n, int m)
{
   int i,j;
   for (i=0; i < n; i++)
   {
      for (j=0; j < m; j++)
      {
			C[i*m + j] = A[i*m + j] - B[i*m + j];
		}
	}
}

BIP::BIP()
{
	printf("BIP constructor\n");
}

BIP::~BIP()
{
}

void BIP::add_demonstration(CInteraction *interaction)
{
	m_interactions.push_back(interaction);
}

// must be called after all demonstrations are added
void BIP::create_initial_ensemble()
{
	printf("%s\n", __func__);

	const double range = 0.1;
	interaction_list::iterator interaction = m_interactions.begin();
	for (int i=0; i < NUM_ENSEMBLE_MEMBERS; i++)
	{
		m_weights[i + NUM_ENSEMBLE_MEMBERS * ENSEMBLE_STATE_PHASE] = 0;
		m_weights[i + NUM_ENSEMBLE_MEMBERS * ENSEMBLE_STATE_PHASE_VEL] = (double)1/(double)(*interaction)->length();
		double variation = (double)rand() / (double)RAND_MAX * range;
		m_weights[i + NUM_ENSEMBLE_MEMBERS * ENSEMBLE_STATE_WEIGHT] = 1;// - (range/2) + variation;
		interaction++;
	}

	print_matrix_double(m_weights, NUM_ENSEMBLE_STATES, NUM_ENSEMBLE_MEMBERS);
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

// ensemble: D x E
void BIP::hx(double *matrix)
{
	const double range = 0.1;
	unsigned int demonstration = 0;
	double sample[NUM_STATE_VARIABLES];

	// look up the sample at this phase directly from the demonstrations
	for (interaction_list::iterator i = m_interactions.begin(); i != m_interactions.end(); i++)
	{
		double variation = (double)rand() / (double)RAND_MAX * range - (range/2.0);
		(*i)->get_sample(m_weights[ENSEMBLE_STATE_PHASE], sample);
		//printf("Got sample from demonstration %i:\n", demonstration);
		//print_matrix_double(sample, 1, NUM_STATE_VARIABLES);

		matrix[NUM_ENSEMBLE_MEMBERS * STATE_VAR_BALL_X + demonstration] = sample[STATE_VAR_BALL_X] *
			m_weights[NUM_ENSEMBLE_MEMBERS * ENSEMBLE_STATE_WEIGHT + demonstration] + variation;
		variation = (double)rand() / (double)RAND_MAX * range - (range/2.0);
		matrix[NUM_ENSEMBLE_MEMBERS * STATE_VAR_BALL_Y + demonstration] = sample[STATE_VAR_BALL_Y] *
			m_weights[NUM_ENSEMBLE_MEMBERS * ENSEMBLE_STATE_WEIGHT + demonstration] + variation;
		variation = (double)rand() / (double)RAND_MAX * range - (range/2.0);
		matrix[NUM_ENSEMBLE_MEMBERS * STATE_VAR_ROBOT_X + demonstration] = sample[STATE_VAR_ROBOT_X] *
			m_weights[NUM_ENSEMBLE_MEMBERS * ENSEMBLE_STATE_WEIGHT + demonstration] + variation;
		demonstration++;
	}
}

/*
 NUM_STATE_VARIABLES * NUM_ENSEMBLE_MEMBERS
*/
void BIP::get_ha_matrix(double *hx, double *ha)
{
	double mean[NUM_STATE_VARIABLES];
	unsigned int vars, index;

	for (vars = 0; vars < NUM_ENSEMBLE_STATES; vars++)
		mean[vars] = 0;

	for (vars = 0; vars < NUM_STATE_VARIABLES; vars++)
	{
		for (index=0; index < NUM_ENSEMBLE_MEMBERS; index++)
		{
			mean[vars] += hx[NUM_ENSEMBLE_MEMBERS * vars + index];
		}
		mean[vars] /= NUM_ENSEMBLE_MEMBERS;
	}

	// ha is the deviation of the predicted state from the mean
	Matrix_Subtract_Vector(hx, mean, ha, NUM_STATE_VARIABLES, NUM_ENSEMBLE_MEMBERS);
}

/* Transforms the given basis space weights to measurement space for the given phase values
	x Vector of dimension T containing the phase values that the basis space weights should be projected at.
 returns matrix D x num_samples
*/
void BIP::get_mean_trajectory(double range_start, double range_end, unsigned int num_samples, double *trajectory)
{
	double phase = range_start;
	double stepping = (range_end - range_start) / (double)num_samples;
	double sample[NUM_STATE_VARIABLES];

	//printf("%s stepping=%f\n", __func__, stepping);

	for (unsigned int sample_index = 0; sample_index < num_samples; sample_index++)
	{
		unsigned int valid_samples = 0;
		trajectory[sample_index + num_samples * STATE_VAR_BALL_X] = 0;
		trajectory[sample_index + num_samples * STATE_VAR_BALL_Y] = 0;
		trajectory[sample_index + num_samples * STATE_VAR_ROBOT_X] = 0;

		// look up the sample at this phase directly from the demonstrations
		for (interaction_list::iterator i = m_interactions.begin(); i != m_interactions.end(); i++)
		{
			(*i)->get_sample(phase, sample);
			//printf("Got sample at phase %f:\n", phase);
			//print_matrix_double(sample, 1, NUM_STATE_VARIABLES);
			apply_weights(valid_samples, sample);

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

		phase += stepping;
	}
}

/*
Performs inference over the given trajectory and returns the most probable future trajectory
*/
/*
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
	return get_mean_trajectory(target_phase, 1, num_samples);
}
*/

void BIP::estimate_state(double sample, double *sensors, double *sensorNoise, double *predictedState)
{
	double *currMean = (double *)calloc(sizeof(double), NUM_ENSEMBLE_STATES);// vector of averages used to derive At
	double *S = (double *)calloc(sizeof(double), NUM_STATE_VARIABLES * NUM_STATE_VARIABLES);		// Innovation co-variance
	double *A_matrix = (double *)calloc(sizeof(double), NUM_ENSEMBLE_STATES * NUM_ENSEMBLE_MEMBERS);
	double *partialKalman = (double *)calloc(sizeof(double), NUM_ENSEMBLE_STATES * NUM_STATE_VARIABLES);
	double *HX_matrix = (double *)calloc(sizeof(double), NUM_STATE_VARIABLES * NUM_ENSEMBLE_MEMBERS);// HtXt|t-1
	double *ha =        (double *)calloc(sizeof(double), NUM_STATE_VARIABLES * NUM_ENSEMBLE_MEMBERS);// HtAt
	double *R = (double *)calloc(sizeof(double), NUM_STATE_VARIABLES * NUM_STATE_VARIABLES);		// random noise
	double *KalmanGain = (double *)calloc(sizeof(double), NUM_ENSEMBLE_STATES * NUM_STATE_VARIABLES);
	double *observations = (double *)calloc(sizeof(double), NUM_STATE_VARIABLES * NUM_ENSEMBLE_MEMBERS);
	double *sensorDiff = (double *)calloc(sizeof(double), NUM_STATE_VARIABLES * NUM_ENSEMBLE_MEMBERS);
	double *KalmanDiff = (double *)calloc(sizeof(double), NUM_ENSEMBLE_STATES * NUM_ENSEMBLE_MEMBERS);

	// build sensor readings for each ensemble member
	add_sensor_noise(sensors, observations, 0.1, NUM_STATE_VARIABLES, NUM_ENSEMBLE_MEMBERS);

	// make forward prediction for each ensemble member
	propagate_ensemble(sample);

	printf("ensemble:\n");
	print_matrix_double(m_weights, NUM_ENSEMBLE_STATES, NUM_ENSEMBLE_MEMBERS);

	get_ensemble_mean(currMean, m_weights);
	//printf("ensemble mean:\n");
	//print_matrix_double(currMean, NUM_ENSEMBLE_STATES, 1);

	// A is the deviation of the current weights from the mean (B x E)
	Matrix_Subtract_Vector(m_weights, currMean, A_matrix, NUM_ENSEMBLE_STATES, NUM_ENSEMBLE_MEMBERS);
	printf("A:\n");
	print_matrix_double(A_matrix, NUM_ENSEMBLE_STATES, NUM_ENSEMBLE_MEMBERS);

	// hx matrix (D x E)
	hx(HX_matrix);
	printf("HX:\n");
	print_matrix_double(HX_matrix, NUM_STATE_VARIABLES, NUM_ENSEMBLE_MEMBERS);

	// ha = HX - avg(HX) (D x E)
	get_ha_matrix(HX_matrix, ha);
	printf("HA:\n");
	print_matrix_double(ha, NUM_STATE_VARIABLES, NUM_ENSEMBLE_MEMBERS);

	// generate some random noise
	generate_noise(R, 0.1, NUM_STATE_VARIABLES, NUM_STATE_VARIABLES);
	//printf("R:\n");
	//print_matrix_double(R, NUM_STATE_VARIABLES, NUM_STATE_VARIABLES);

	// S is the innovation covariance (D x E . E x D = D x D)
	cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasTrans,
		NUM_STATE_VARIABLES, NUM_STATE_VARIABLES, NUM_ENSEMBLE_MEMBERS, (double)1/(double)(NUM_ENSEMBLE_MEMBERS-1),
		ha, NUM_ENSEMBLE_MEMBERS,
		ha, NUM_ENSEMBLE_MEMBERS, 0, S, NUM_STATE_VARIABLES);
	Matrix_Add_Matrix(S, R, S, NUM_STATE_VARIABLES, NUM_STATE_VARIABLES);
	printf("S:\n");
	print_matrix_double(S, NUM_STATE_VARIABLES, NUM_STATE_VARIABLES);

	// Calculate S-inverse
	int pivots[NUM_STATE_VARIABLES];
	LAPACKE_dgetrf(LAPACK_ROW_MAJOR, NUM_STATE_VARIABLES, NUM_STATE_VARIABLES, S, NUM_STATE_VARIABLES, pivots);
	LAPACKE_dgetri(LAPACK_ROW_MAJOR, NUM_STATE_VARIABLES, S, NUM_STATE_VARIABLES, pivots);
	//printf("S-inv:\n");
	//print_matrix_double(S, NUM_STATE_VARIABLES, NUM_STATE_VARIABLES);

	// partial Kalman (B x E . E x D = B x D)
	cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasTrans,
		NUM_ENSEMBLE_STATES, NUM_STATE_VARIABLES, NUM_ENSEMBLE_MEMBERS, (double)1/(double)(NUM_ENSEMBLE_MEMBERS-1),
		A_matrix, NUM_ENSEMBLE_MEMBERS,
		ha, NUM_ENSEMBLE_MEMBERS, 0, partialKalman, NUM_STATE_VARIABLES);
	printf("partial K:\n");
	print_matrix_double(partialKalman, NUM_ENSEMBLE_STATES, NUM_STATE_VARIABLES);

	// Calculate Kalman gain (B x D . D x D = B x D)
	cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans,
		NUM_ENSEMBLE_STATES, NUM_STATE_VARIABLES, NUM_STATE_VARIABLES, (double)1/(double)(NUM_ENSEMBLE_MEMBERS-1),
		S, NUM_STATE_VARIABLES,
		partialKalman, NUM_STATE_VARIABLES, 0, KalmanGain, NUM_STATE_VARIABLES);
	for (int i=0; i < NUM_ENSEMBLE_STATES; i++)
	KalmanGain[NUM_ENSEMBLE_STATES * ENSEMBLE_STATE_PHASE + i] = 0;
	printf("K:\n");
	print_matrix_double(KalmanGain, NUM_ENSEMBLE_STATES, NUM_STATE_VARIABLES);

	// Calculate difference (B x D . D x E = B x E)
	Matrix_Subtract_Matrix(observations, HX_matrix, sensorDiff, NUM_STATE_VARIABLES, NUM_ENSEMBLE_MEMBERS);
	printf("observations - HX:\n");
	print_matrix_double(sensorDiff, NUM_STATE_VARIABLES, NUM_ENSEMBLE_MEMBERS);
	cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans,
		NUM_ENSEMBLE_STATES, NUM_ENSEMBLE_MEMBERS, NUM_STATE_VARIABLES, 1,
		KalmanGain, NUM_STATE_VARIABLES,
		sensorDiff, NUM_ENSEMBLE_MEMBERS, 0, KalmanDiff, NUM_ENSEMBLE_MEMBERS);
	printf("K diff:\n");
	print_matrix_double(KalmanDiff, NUM_ENSEMBLE_STATES, NUM_ENSEMBLE_MEMBERS);

	// Update ensemble (B x E += B x E)
	Matrix_Add_Matrix(m_weights, KalmanDiff, m_weights, NUM_ENSEMBLE_STATES, NUM_ENSEMBLE_MEMBERS);

	// apply weights to state
	get_weighted_mean(predictedState);

	free(currMean);
	free(ha);
	free(S);
	free(A_matrix);
	free(partialKalman);
	free(R);
	free(KalmanGain);
	free(sensorDiff);
	free(KalmanDiff);
	free(HX_matrix);
	free(observations);
}

/*
	state ensemble B x E
	mean Vector of dimension B
*/
void BIP::get_ensemble_mean(double *mean, double *ensemble)
{
	unsigned int index;
	unsigned int vars;

	if (!ensemble)
		printf("No ensemble\n");

	for (unsigned int vars = 0; vars < NUM_ENSEMBLE_STATES; vars++)
		mean[vars] = 0;

	for (vars = 0; vars < NUM_ENSEMBLE_STATES; vars++)
	{
		for (index=0; index < NUM_ENSEMBLE_MEMBERS; index++)
		{
			mean[vars] += ensemble[NUM_ENSEMBLE_MEMBERS * vars + index];
		}
		mean[vars] /= NUM_ENSEMBLE_MEMBERS;
	}
}

void BIP::generate_noise(double *matrix, double range, int n, int m)
{
	int row, col;
	for (row = 0; row < n; row++)
	{
		for (col=0; col < m; col++)
		{
			matrix[row * m + col] = ((double)rand()/(double)RAND_MAX) * range - range/(double)2;
		}
	}
}

/* update the phase based on phase velocity */
void BIP::propagate_ensemble(double sample)
{
	printf("Propagating at phase: ");
	unsigned int i;
	for (i=0; i < NUM_ENSEMBLE_MEMBERS; i++)
	{
		double phase = sample * m_weights[i + NUM_ENSEMBLE_MEMBERS * ENSEMBLE_STATE_PHASE_VEL];
		//printf("%f ", phase * m_weights[i + NUM_ENSEMBLE_MEMBERS * ENSEMBLE_STATE_PHASE_VEL]);
		printf("%f ", phase);

		//m_weights[i + NUM_ENSEMBLE_MEMBERS * ENSEMBLE_STATE_PHASE] += 
		//	(m_weights[i + NUM_ENSEMBLE_MEMBERS * ENSEMBLE_STATE_PHASE_VEL] * phase);
		m_weights[i + NUM_ENSEMBLE_MEMBERS * ENSEMBLE_STATE_PHASE] = phase; 

		// make sure the phase does not exceed the limits
		if (m_weights[i + NUM_ENSEMBLE_MEMBERS * ENSEMBLE_STATE_PHASE] < 0)
			m_weights[i + NUM_ENSEMBLE_MEMBERS * ENSEMBLE_STATE_PHASE] = 0;

		if (m_weights[i + NUM_ENSEMBLE_MEMBERS * ENSEMBLE_STATE_PHASE] > 1)
			m_weights[i + NUM_ENSEMBLE_MEMBERS * ENSEMBLE_STATE_PHASE] = 1;
	}
	
	putc('\n', stdout);
}

void BIP::add_sensor_noise(double *sensors, double *obs, double range, int m, int n)
{
	unsigned int row, col;
	for (row=0; row < m; row++)
	{
		for (col=0; col < n; col++)
		{
			obs[row * n + col] = sensors[row] + ((double)rand()/(double)RAND_MAX) * range - range/(double)2;
		}
	}
}

void BIP::get_weighted_mean(double *matrix)
{
	unsigned int state;
	unsigned int demonstration = 0;
	double sample[NUM_STATE_VARIABLES];

	for (state=0; state < NUM_STATE_VARIABLES; state++)
		matrix[state] = 0;

	// look up the sample at this phase directly from the demonstrations
	for (interaction_list::iterator i = m_interactions.begin(); i != m_interactions.end(); i++)
	{
		(*i)->get_sample(m_weights[ENSEMBLE_STATE_PHASE], sample);
		//printf("Got sample from demonstration %i:\n", demonstration);
		//print_matrix_double(sample, 1, NUM_STATE_VARIABLES);

		apply_weights(demonstration, sample);
		matrix[STATE_VAR_BALL_X] += sample[STATE_VAR_BALL_X];
		matrix[STATE_VAR_BALL_Y] += sample[STATE_VAR_BALL_Y];
		matrix[STATE_VAR_ROBOT_X] += sample[STATE_VAR_ROBOT_X];
		demonstration++;
	}

	for (state=0; state < NUM_STATE_VARIABLES; state++)
		matrix[state] /= demonstration;
}

void BIP::apply_weights(int member, double *sample)
{
	sample[STATE_VAR_BALL_X] *= m_weights[NUM_ENSEMBLE_MEMBERS * ENSEMBLE_STATE_WEIGHT + member];
	sample[STATE_VAR_BALL_Y] *= m_weights[NUM_ENSEMBLE_MEMBERS * ENSEMBLE_STATE_WEIGHT + member];
	sample[STATE_VAR_ROBOT_X] *= m_weights[NUM_ENSEMBLE_MEMBERS * ENSEMBLE_STATE_WEIGHT + member];
}
