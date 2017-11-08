/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 *  Modified for Udacity Self Driving Car Project on Nov 3, 2017
 *			Author: Ramesh Chukka
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

const float VSV = 0.01;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	if (!is_initialized){
		num_particles = 100; //as tried in particle filter lessons

		//Random generator
		default_random_engine gen;
		// normal (Gaussian) distribution for x, y, and theta
		// std for x, y, and theta are available in std[0], std[1] and std[2] respectively
		normal_distribution<double> dist_x(x, std[0]);
		normal_distribution<double> dist_y(y, std[1]);
		normal_distribution<double> dist_theta(theta, std[2]);

		for (int i = 0; i < num_particles; ++i) {
			double sample_x, sample_y, sample_theta;

			// TODO: Sample  and from these normal distrubtions like this:
			// sample_x = dist_x(gen);
			// where "gen" is the random engine initialized earlier.
	    	sample_x = dist_x(gen);
			sample_y = dist_y(gen);
			sample_theta = dist_theta(gen);

			Particle p;
			p.id = i;
			p.x = sample_x;
			p.y = sample_y;
			p.theta = sample_theta;
			// set the weight to 1 as suggested in TODO
			p.weight = 1;
			//store the particle info in particles vector
			particles.push_back(p);
			weights.push_back(1.0f);

			// Print your samples to the terminal.
			cout << "Sample " << i + 1 << " " << sample_x << " " << sample_y << " " << sample_theta << endl;
		}
		is_initialized = true;
	}

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	//Random generator
	default_random_engine gen;

	// normal (Gaussian) distribution for x, y, theta with mean as 0
	// std for x, y, and theta are available in std_pos[0], std_pos[1] and std_pos[2] respectively
	// TODO: Sample  and from these normal distrubtions like this:
	// sample_x = dist_x(gen);
	// where "gen" is the random engine initialized earlier.
	normal_distribution<double> dist_x(0, std_pos[0]);
	normal_distribution<double> dist_y(0, std_pos[1]);
	normal_distribution<double> dist_theta(0, std_pos[2]);

	for (int i = 0; i < num_particles; ++i) {
		double sample_theta;

		sample_theta = particles[i].theta;
		//if Xsig_aug[4] != 0  (i.e yawdot)
	      if (std::fabs(yaw_rate) > VSV){
	         particles[i].x += (velocity/yaw_rate)*((sin(sample_theta+yaw_rate*delta_t))-sin(sample_theta)) ;
	         particles[i].y += (velocity/yaw_rate)*((-cos(sample_theta+yaw_rate*delta_t))+cos(sample_theta)) ;
			 particles[i].theta += yaw_rate*delta_t;

	      }else{
	         particles[i].x += velocity*cos(sample_theta)*delta_t;
	         particles[i].y += velocity*sin(sample_theta)*delta_t;
	      }

		  //Add random noise to the particle measurements
		  particles[i].x += dist_x(gen);
		  particles[i].y += dist_y(gen);
		  particles[i].theta += dist_theta(gen);
	}

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.

	//Assume both predicted and observation msmts are in the same coordinate system (MAP's?)
	int min_pid = 0;

	//Traverse thru each of observations
	for (int nObs = 0; nObs < observations.size(); ++nObs){

		//set the initial min value to maximum number
		double nMinValue = std::numeric_limits<double>::max();

		//Get the nObs th observations
		LandmarkObs obs = observations[nObs];

		//Now look thru all of the predicted msmts for closest
		for (int nPred = 0; nPred < predicted.size(); ++nPred){
			//Get the nObs th observations
			LandmarkObs pred = predicted[nPred];

			/*if ((pred != nullptr) || (obs != nullptr))*/{
				//Find the distance between obs and pred
				double dist1 = dist(obs.x, obs.y, pred.x, pred.y);
				if (dist1 < nMinValue){
					nMinValue = dist1;
					min_pid = pred.id;
				}
			}
		}
		//Now set the closest predicted landmark to the obs
		observations[nObs].id = min_pid;

	}

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	//For each particles
	for (int i = 0; i < num_particles; ++i) {

		// Weed out the landmakrs that are not within range of the sensor from the particle
		double p_x = particles[i].x;
		double p_y = particles[i].y;
		double p_theta = particles[i].theta;

		vector<LandmarkObs> withinRange_predictions;

		for (int j = 0; j < map_landmarks.landmark_list.size(); ++j){

			Map::single_landmark_s lm = map_landmarks.landmark_list[j];

			float jm_x = lm.x_f;
			float jm_y = lm.y_f;

			//find the distance between the particle and the land map_landmarks
			double dist1 = dist(p_x, p_y, jm_x, jm_y);
			if (dist1 <= sensor_range){
				withinRange_predictions.push_back(LandmarkObs{lm.id_i, jm_x, jm_y});
			}
		}

		//Now that we have the predictions in vehicle space - convert the observation also to map coordinte space
		//New observations in map space
		vector<LandmarkObs> obs_in_map_space;
		for (int k = 0; k < observations.size(); ++k){
			double map_x = observations[k].x*cos(p_theta) - observations[k].y*sin(p_theta) + p_x;
			double map_y = observations[k].x*sin(p_theta) + observations[k].y*cos(p_theta) + p_y;

			obs_in_map_space.push_back(LandmarkObs{observations[k].id, (float)map_x, (float)map_y});

		}

		//Find the closest observations to the predictions
		dataAssociation(withinRange_predictions, obs_in_map_space);

		//total_prob_of_obs
		double total_prob_obs = 1.0f;

		//Find the weights using multi variate gaussin
		for (int l = 0; l < obs_in_map_space.size(); ++l){
			double map_x = obs_in_map_space[l].x;
			double map_y = obs_in_map_space[l].y;

			//std::cout << "LOG: obs_in_map_space: obs:"  << " " << obs_in_map_space[l].id << " " << obs_in_map_space[l].x << " " << obs_in_map_space[l].y << std::endl;

			LandmarkObs lm = return_matched_obs_for_id(obs_in_map_space[l].id, withinRange_predictions);

			//std::cout << "LOG: after return_matched_obs_for_id:"  << " " << lm.id << " " << lm.x << " " << lm.y << std::endl;

			double mv_g_prob = return_multivariate_gaussian(map_x, map_y, std_landmark[0], std_landmark[1], lm.x, lm.y);

			//cout << "LOG: multi variate prob: " << mv_g_prob << endl;
			//Particle's final weight is the multiplication of all mv prob of all observations.
			total_prob_obs *= mv_g_prob;;


		}
		particles[i].weight = total_prob_obs;
		// Add weights to weight vector
		weights[i] = particles[i].weight;
	}

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	/* ******** Sebastan's Particle filter lesson - exercise code  in python *********
		from random import uniform, randint
		p3 = []
		beta = 0
		max_w = max(w)
		w_index = randint(0, N)
		for i in range(N):
			beta = uniform(0, 2*max_w)
			while(w[w_index] < beta):
				beta = beta - w[w_index]
				w_index += 1
				w_index = w_index % N
			p3.append(p[w_index])
		p = p3
	* ************************************************************************** */
	// Set of new particles that came out of selection with replacement
	std::vector<Particle> newParticles;

	double beta = 0.0;

	double max_w = *std::max_element(weights.begin(), weights.end());
	//Random generator
	default_random_engine gen;
	//uniform int distribution http://en.cppreference.com/w/cpp/numeric/random/uniform_int_distribution
	std::uniform_int_distribution<> dis(0, num_particles-1);
	int w_index = dis(gen);

	//Uniform real distribution http://www.cplusplus.com/reference/random/uniform_real_distribution/
	std::uniform_real_distribution<double> real_dis(0.0, 2*max_w);

	for (int i; i < num_particles; ++i){
		beta = real_dis(gen);

		while(weights[w_index] < beta){
			beta = beta - weights[w_index];
			w_index = (w_index+1)%num_particles;
		}
		newParticles.push_back(particles[w_index]);
	}
	particles = newParticles;
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
