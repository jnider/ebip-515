#include <cblas.h>
//#include <f77blas.h>
#include <lapacke.h>
#include "enkf.h"

extern void print_matrix_double(double *A, int n, int m);

EnsembleKalmanFilter::EnsembleKalmanFilter(BIP *basis_model) : m_basis_model(basis_model), m_ensemble(NULL)
{
	m_ensemble = (double*)calloc(sizeof(double), NUM_STATE_VARIABLES * NUM_ENSEMBLE_MEMBERS);
}

EnsembleKalmanFilter::~EnsembleKalmanFilter() {};

/*
    Calculates the unbiased sample mean of the ensemble.

	ensemble E x D
	mean Vector of dimension D (=NUM_STATE_VARIABLES)
*/
void EnsembleKalmanFilter::get_ensemble_mean(double *mean, double *ensemble)
{
	unsigned int total = 0;
	unsigned int index;
	unsigned int vars;

	for (unsigned int vars = 0; vars < NUM_STATE_VARIABLES; vars++)
		mean[vars] = 0;

	if (!ensemble)
		printf("No ensemble\n");

	for (unsigned int vars = 0; vars < NUM_STATE_VARIABLES; vars++)
	{
		for (index=0; index < NUM_ENSEMBLE_MEMBERS; index++)
		{
			mean[vars] += ensemble[index + NUM_STATE_VARIABLES * vars];
		}
	}

	// calculate average
	for (unsigned int vars = 0; vars < NUM_STATE_VARIABLES; vars++)
		mean[vars] /= NUM_ENSEMBLE_MEMBERS;
}

/*
    #   Calculates the unbiased sample covariance of the ensemble.
    #   The formula used to compute the covariance is: $$ cov(\\boldsymbol{X}) = \\frac{1}{E - 1} \\boldsymbol{A} \\boldsymbol{A}^{T}, \\qquad \\boldsymbol{A} = \\boldsymbol{X} - \\bar{\\boldsymbol{X}}, $$
    #   where \f$ cov(\boldsymbol{X}) \in \mathbb{R}^{N+1+B \times N+1+B}. \f$
    #
    #   @param ensemble The ensemble from which to calculate the sample covariance. If none, uses the internal state ensemble.
    #
    #   @returns Matrix of dimension N+1+B x N+1+B containing the sample covariance.
    #
    def get_ensemble_covariance(self, ensemble = None):
        mean = self.get_ensemble_mean(ensemble)

        if(ensemble is None):
            deviation = (self.ensemble.T - mean).T
        else:
            deviation = (ensemble.T - mean).T

        # Ensemble variance is the square deviation divided by the ensemble size - 1. [1]
        return np.dot(deviation, deviation.T) / (self.ensemble_size - 1.0)
*/
void EnsembleKalmanFilter::get_ensemble_covariance(double *covar)
{
/*
	cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasTrans,
		num_members, num_members, num_vars, (double)1/(double)(num_members-1),
		deviation, num_vars,
		deviation, num_vars, 0, S, num_members);
*/
}

void EnsembleKalmanFilter::get_projected_mean_covariance()
{
}

void EnsembleKalmanFilter::h(double *state)
{
	printf("%s\n", __func__);
/*
	double new_state[NUM_STATE_VARIABLES];
	if (!state)
	{
		printf("bad state\n");
		return;
	}
	if (!m_basis_model)
	{
		printf("No basis model\n");
		return;
	}
*/
	//m_basis_model->apply_coefficients(state[STATE_VAR_PHASE], state, new_state);
}

/*
   This method performs simultaneous localization of both time (phase) and space (basis weights).
   This is a recursive call, which means it updates the internal state estimate based on the given observations.
   For each observation given, two steps are performed recursively:
   First, the current ensemble is propagated forward in time in what is known as the prediction step.
   Note that in this case, we only apply the dot product to the first N+1 dimensions of the state. This is for computational efficiency as only the constant velocity phase system has a non-zero transition.
   Next, we integrate the observations into the current ensemble in the update step.
   Lastly, the sample mean and covariance of the ensemble are returned.
   At the end of both the prediction and update steps the internal phase value is clipped such that it falls within the range [0, 1].

   measurement Matrix of dimension T x D containing observations, where T is the number of timesteps that have been observed since the last call to localize() and D is the dimension of the measurement space.
   measurement_noise Matrix of dimension D x D containing the measurement noise for the given set of measurements.
   Scalar value containing the inferred phase, Vector of dimension D (or N+1+D if return_phase_variance is True) containing inferred mean, Matrix of dimension D x D (or N+1+D x N+1+D if return_phase_variance is True).
*/
void EnsembleKalmanFilter::localize(double *measurement, double *noise, double *phase, double *mean, double *covar)
{
	unsigned int index;
	unsigned int var;
	double *hx_matrix = (double*)calloc(sizeof(double), NUM_STATE_VARIABLES * NUM_ENSEMBLE_MEMBERS);
	double *A = (double*)calloc(sizeof(double), NUM_STATE_VARIABLES * NUM_ENSEMBLE_MEMBERS);
	double *ha = (double*)calloc(sizeof(double), NUM_STATE_VARIABLES * NUM_ENSEMBLE_MEMBERS);
	double *S = (double*)calloc(sizeof(double), NUM_STATE_VARIABLES * NUM_STATE_VARIABLES);
	double *aha = (double*)calloc(sizeof(double), NUM_STATE_VARIABLES * NUM_STATE_VARIABLES);
	double *Sinv = (double*)calloc(sizeof(double), NUM_STATE_VARIABLES * NUM_STATE_VARIABLES);
	double *K = (double*)calloc(sizeof(double), NUM_STATE_VARIABLES * NUM_STATE_VARIABLES);

	printf("%s\n", __func__);

	get_ensemble_mean(mean, m_ensemble);
	printf("Current mean:\n");
	print_matrix_double(mean, NUM_STATE_VARIABLES, 1);

	// Calculate HX by getting next state for each ensemble member and multiplying by current weights
	printf("Calculating HX\n");

	memcpy(hx_matrix, m_ensemble, sizeof(double) * NUM_STATE_VARIABLES * NUM_ENSEMBLE_MEMBERS);

/*
	// calculate the estimated averages while getting the next state estimation
	for (index=0; index < NUM_ENSEMBLE_MEMBERS; index++)
	{
		double x=0, y=0;
		m_ensemble[index].phase = ((double)abs_ns/1000000) * m_ensemble[index].phase_velocity_ms;
		(*i)->nonlinear_fn(m_ensemble[index].phase, SENSOR_BALL, &x, &y);

		nextEnsemble[index * NUM_STATE_VARIABLES + STATE_VAR_PHASE] = phase;
		nextEnsemble[index * NUM_STATE_VARIABLES + STATE_VAR_PHASE_VEL] = 0;
		nextEnsemble[index * NUM_STATE_VARIABLES + STATE_VAR_BALL_X] = x;
		nextEnsemble[index * NUM_STATE_VARIABLES + STATE_VAR_BALL_Y] = y;
		nextAverage[STATE_VAR_PHASE] += phase;
		nextAverage[STATE_VAR_BALL_X] += x;
		nextAverage[STATE_VAR_BALL_Y] += y;

		(*i)->nonlinear_fn(m_ensemble[index].phase, SENSOR_ROBOT, &x, &y);
		nextEnsemble[index * NUM_STATE_VARIABLES + STATE_VAR_ROBOT_X] = x;
		nextAverage[STATE_VAR_ROBOT_X] += x;
		i++;
	}
*/
	// Calculate A (DxE)
	// a = (self.ensemble.T - mean).T
	// TODO use dgemv()
	for (index = 0; index < NUM_ENSEMBLE_MEMBERS; index++)
	{
		for (var = 0; var < NUM_STATE_VARIABLES; var++)
			A[var * NUM_ENSEMBLE_MEMBERS + index] = abs(m_ensemble[var * NUM_ENSEMBLE_MEMBERS + index] - mean[var]);
	}
	printf("A:\n");
	print_matrix_double(A, NUM_STATE_VARIABLES, NUM_ENSEMBLE_MEMBERS);
	

	// Calculate HA (DxE)
	// ha = (hx_matrix.T - (np.sum(hx_matrix, axis = 1) / self.ensemble_size)).T

	// Calculate S (DxD)
	// covariance_residual = (1.0 / (self.ensemble_size - 1.0)) * np.dot(ha, ha.T) + measurement_noise
	cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasTrans,
		NUM_STATE_VARIABLES, NUM_STATE_VARIABLES, NUM_STATE_VARIABLES, ((double)1.0 / (double)(NUM_ENSEMBLE_MEMBERS - 1.0)),
		ha, NUM_STATE_VARIABLES,
		ha, NUM_STATE_VARIABLES, 0, S, NUM_STATE_VARIABLES);
	printf("S:\n");
	print_matrix_double(S, NUM_STATE_VARIABLES, NUM_STATE_VARIABLES);

	// Calculate K
	// kalman_gain = (1.0 / (self.ensemble_size - 1.0)) * np.dot(a, ha.T).dot(np.linalg.pinv(covariance_residual))
	int rows = NUM_STATE_VARIABLES;
	int cols = NUM_STATE_VARIABLES;
	int pivots[NUM_STATE_VARIABLES];
	int info;
	double *sol;
	int sdim = 1;
	char trans = 'A';

	// Calculate S-inverse
	LAPACKE_dgetrf(LAPACK_ROW_MAJOR, NUM_STATE_VARIABLES, NUM_STATE_VARIABLES, S, NUM_STATE_VARIABLES, pivots);
	//dgetrs_(&trans, &rows, &cols, S, &cols, pivots, sol, &sdim, &info);
	LAPACKE_dgetri(LAPACK_ROW_MAJOR, NUM_STATE_VARIABLES, S, NUM_STATE_VARIABLES, pivots);
	printf("Sinv:\n");
	print_matrix_double(S, NUM_STATE_VARIABLES, NUM_STATE_VARIABLES);

	// a x ha.T (D x E) x (E x D) = D x D
	cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasTrans,
		NUM_STATE_VARIABLES, NUM_STATE_VARIABLES, NUM_STATE_VARIABLES, 1,
		A, NUM_STATE_VARIABLES,
		ha, NUM_STATE_VARIABLES, 0, aha, NUM_STATE_VARIABLES);
	printf("a x ha.T:\n");
	print_matrix_double(aha, NUM_STATE_VARIABLES, NUM_STATE_VARIABLES);

	cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasTrans,
		NUM_STATE_VARIABLES, NUM_STATE_VARIABLES, NUM_STATE_VARIABLES, ((double)1.0 / (double)(NUM_ENSEMBLE_MEMBERS - 1.0)),
		aha, NUM_STATE_VARIABLES,
		Sinv, NUM_STATE_VARIABLES, 0, K, NUM_STATE_VARIABLES);
	printf("K:\n");
	print_matrix_double(K, NUM_STATE_VARIABLES, NUM_STATE_VARIABLES);

	// Zero out the Kalman gain entries for the non-active DoFs. Since we aren't considering them we don't want them to affect the update process.

	// Update the ensemble
	// self.ensemble += kalman_gain.dot(noisy_observations.T - hx_matrix)
	cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasTrans,
		NUM_STATE_VARIABLES, NUM_STATE_VARIABLES, NUM_STATE_VARIABLES, 1,
		K, NUM_STATE_VARIABLES,
		hx_matrix, NUM_STATE_VARIABLES, 0, m_ensemble, NUM_STATE_VARIABLES);

	// self.ensemble[0, self.ensemble[0, :] > 1.0] = 1.0
	// self.ensemble[0, self.ensemble[0, :] < 0.0] = 0.0

	//expected_value = self.get_ensemble_mean()
	//get_ensemble_mean(mean);
	//expected_variance = self.get_ensemble_covariance()
	//get_ensemble_covariance(var);

	free(hx_matrix);
	free(A);
	free(aha);
	free(S);
	free(Sinv);
	free(K);
}
