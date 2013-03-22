/*
(The MIT License.)

Copyright (c) 2009, Kevin Lacker.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

/*
 * This is a Java port of original C code by Kevin Lacker.
 * https://github.com/lacker/ikalman
 * 
 * Ported by Andrey Novikov, 2013
 */

package com.androzic.utils;

/* Refer to http://en.wikipedia.org/wiki/Kalman_filter for
 mathematical details. The naming scheme is that variables get names
 that make sense, and are commented with their analog in
 the Wikipedia mathematical notation.
 This Kalman filter implementation does not support controlled
 input.
 (Like knowing which way the steering wheel in a car is turned and
 using that to inform the model.)
 Vectors are handled as n-by-1 matrices.
 TODO: comment on the dimension of the matrices */

public class KalmanFilter {
	/* k */
	int timestep;

	/* These parameters define the size of the matrices. */
	int state_dimension, observation_dimension;

	/* This group of matrices must be specified by the user. */
	/* F_k */
	Matrix state_transition;
	/* H_k */
	Matrix observation_model;
	/* Q_k */
	Matrix process_noise_covariance;
	/* R_k */
	Matrix observation_noise_covariance;

	/* The observation is modified by the user before every time step. */
	/* z_k */
	Matrix observation;

	/* This group of matrices are updated every time step by the filter. */
	/* x-hat_k|k-1 */
	Matrix predicted_state;
	/* P_k|k-1 */
	Matrix predicted_estimate_covariance;
	/* y-tilde_k */
	Matrix innovation;
	/* S_k */
	Matrix innovation_covariance;
	/* S_k^-1 */
	Matrix inverse_innovation_covariance;
	/* K_k */
	Matrix optimal_gain;
	/* x-hat_k|k */
	Matrix state_estimate;
	/* P_k|k */
	Matrix estimate_covariance;

	/* This group is used for meaningless intermediate calculations */
	Matrix vertical_scratch;
	Matrix small_square_scratch;
	Matrix big_square_scratch;

	public KalmanFilter(int state_dimension, int observation_dimension) {
		timestep = 0;
		this.state_dimension = state_dimension;
		this.observation_dimension = observation_dimension;

		state_transition = new Matrix(state_dimension, state_dimension);
		observation_model = new Matrix(observation_dimension, state_dimension);
		process_noise_covariance = new Matrix(state_dimension, state_dimension);
		observation_noise_covariance = new Matrix(observation_dimension, observation_dimension);

		observation = new Matrix(observation_dimension, 1);

		predicted_state = new Matrix(state_dimension, 1);
		predicted_estimate_covariance = new Matrix(state_dimension,	state_dimension);
		innovation = new Matrix(observation_dimension, 1);
		innovation_covariance = new Matrix(observation_dimension, observation_dimension);
		inverse_innovation_covariance = new Matrix(observation_dimension, observation_dimension);
		optimal_gain = new Matrix(state_dimension, observation_dimension);
		state_estimate = new Matrix(state_dimension, 1);
		estimate_covariance = new Matrix(state_dimension, state_dimension);

		vertical_scratch = new Matrix(state_dimension, observation_dimension);
		small_square_scratch = new Matrix(observation_dimension,observation_dimension);
		big_square_scratch = new Matrix(state_dimension, state_dimension);
	}

	/*
	 * Runs one timestep of prediction + estimation.
	 * 
	 * Before each time step of running this, set f.observation to be the next
	 * time step's observation.
	 * 
	 * Before the first step, define the model by setting: f.state_transition
	 * f.observation_model f.process_noise_covariance
	 * f.observation_noise_covariance
	 * 
	 * It is also advisable to initialize with reasonable guesses for
	 * f.state_estimate f.estimate_covariance
	 */
	void update() {
		predict();
		estimate();
	}

	/* Just the prediction phase of update. */
	void predict() {
		timestep++;

		/* Predict the state */
		Matrix.multiply_matrix(state_transition, state_estimate, predicted_state);

		/* Predict the state estimate covariance */
		Matrix.multiply_matrix(state_transition, estimate_covariance, big_square_scratch);
		Matrix.multiply_by_transpose_matrix(big_square_scratch, state_transition, predicted_estimate_covariance);
		Matrix.add_matrix(predicted_estimate_covariance, process_noise_covariance, predicted_estimate_covariance);
	}

	/* Just the estimation phase of update. */
	void estimate() {
		/* Calculate innovation */
		Matrix.multiply_matrix(observation_model, predicted_state, innovation);
		Matrix.subtract_matrix(observation, innovation, innovation);

		/* Calculate innovation covariance */
		Matrix.multiply_by_transpose_matrix(predicted_estimate_covariance, observation_model, vertical_scratch);
		Matrix.multiply_matrix(observation_model, vertical_scratch, innovation_covariance);
		Matrix.add_matrix(innovation_covariance, observation_noise_covariance, innovation_covariance);

		/*
		 * Invert the innovation covariance. Note: this destroys the innovation
		 * covariance. TODO: handle inversion failure intelligently.
		 */
		Matrix.destructive_invert_matrix(innovation_covariance, inverse_innovation_covariance);

		/*
		 * Calculate the optimal Kalman gain. Note we still have a useful
		 * partial product in vertical scratch from the innovation covariance.
		 */
		Matrix.multiply_matrix(vertical_scratch, inverse_innovation_covariance, optimal_gain);

		/* Estimate the state */
		Matrix.multiply_matrix(optimal_gain, innovation, state_estimate);
		Matrix.add_matrix(state_estimate, predicted_state, state_estimate);

		/* Estimate the state covariance */
		Matrix.multiply_matrix(optimal_gain, observation_model, big_square_scratch);
		Matrix.subtract_from_identity_matrix(big_square_scratch);
		Matrix.multiply_matrix(big_square_scratch, predicted_estimate_covariance, estimate_covariance);
	}
}
