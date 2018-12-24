/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
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

//#include <cmath>
#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) 
{
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	
  	num_particles = 100;
	
	std::default_random_engine gen;
	std::normal_distribution<double> N_x(x, std[0]);
	std::normal_distribution<double> N_y(y, std[1]);
	std::normal_distribution<double> N_theta(theta, std[2]);
	
	for(int i = 0; i < num_particles; i++)
	{
		Particle particle;
		particle.id = i;
		particle.x = N_x(gen);
		particle.y = N_y(gen);
		particle.theta = N_theta(gen);
      	particle.weight = 1;
      
		particles.push_back(particle);
		weights.push_back(1);
	}
	
	is_initialized = true;
}
void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) 
{
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	std::default_random_engine gen;
	for(int i = 0; i<num_particles; i++)
	{
		double new_x;
		double new_y;
		double new_theta;
		
		if(yaw_rate == 0)
		{
			new_x = particles[i].x + velocity * delta_t*cos(particles[i].theta);
			new_y = particles[i].y + velocity * delta_t*sin(particles[i].theta);
			new_theta = particles[i].theta;
		}
		else
		{
			new_x = particles[i].x + velocity/yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
			new_y = particles[i].y + velocity/yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
			new_theta = particles[i].theta + yaw_rate * delta_t;			
		}
		std::normal_distribution<double> N_x(new_x, std_pos[0]);
		std::normal_distribution<double> N_y(new_y, std_pos[1]);
		std::normal_distribution<double> N_theta(new_theta, std_pos[2]);
		
		particles[i].x = N_x(gen);
		particles[i].y = N_y(gen);
		particles[i].theta = N_theta(gen);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) 
{
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	unsigned int obs_index = 0;
  	unsigned int pred_index = 0;
  	double min_dist = 1000000.0;
  	double dist = 1000000.0;

  	for(obs_index = 0; obs_index<observations.size(); obs_index++)
  	{
  	  	for(pred_index = 0; pred_index<observations.size(); pred_index++)
  	  	{
  	  		dist = sqrt(pow((predicted[pred_index].x - observations[obs_index].x),2) + pow((predicted[pred_index].y - observations[obs_index].y),2));
  	  		if(dist < min_dist)
  	  		{
  	  			min_dist = dist;
  	  			observations[obs_index].id = predicted[pred_index].id;
  	  		}
  	  	}
  	}
}


void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) 
{
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

	vector<int> associations;
	vector<double> sense_x;
	vector<double> sense_y;
	
	vector<LandmarkObs> trans_observations;
	LandmarkObs obs;
	vector<LandmarkObs> landmarks;

	for (int p_indx = 0; p_indx < particles.size(); p_indx++)
	{
		for (int o_indx = 0; o_indx < observations.size(); o_indx++)
		{
			LandmarkObs trans_obs;
			obs = observations[o_indx];

			// perform the space transformation from vehicle to map
			trans_obs.x = particles[p_indx].x + (obs.x * cos(particles[p_indx].theta) - obs.y * sin(particles[p_indx].theta));
			trans_obs.y = particles[p_indx].y + (obs.x * sin(particles[p_indx].theta) + obs.y * cos(particles[p_indx].theta));
			trans_observations.push_back(trans_obs);
		}
		
		particles[p_indx].weight = 1.0;
		
		for (int landmark_indx = 0; landmark_indx < map_landmarks.landmark_list.size(); landmark_indx++)
		{
			landmarks[landmark_indx].id = map_landmarks.landmark_list[landmark_indx].id_i;
			landmarks[landmark_indx].x = map_landmarks.landmark_list[landmark_indx].x_f;
			landmarks[landmark_indx].y = map_landmarks.landmark_list[landmark_indx].y_f;
		}
		
		dataAssociation(landmarks, trans_observations);

		particles[p_indx].weight = 

	}
}

void ParticleFilter::resample() 
{
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
  
  std::default_random_engine gen;
  
  // Get weights and find max weight.
  vector<double> weights;
  double max_weight = numeric_limits<double>::min();
  for(int p_indx = 0; p_indx < num_particles; p_indx++) 
  {
    weights.push_back(particles[p_indx].weight);
    if ( particles[p_indx].weight > max_weight ) 
    {
      max_weight = particles[p_indx].weight;
    }
  }

  // Creating distributions.
  uniform_real_distribution<double> weight_dist(0.0, max_weight);
  discrete_distribution<int> index_dist(0, num_particles - 1);

  // Generating index.
  int index = index_dist(gen);

  double beta = 0.0;

  // the wheel
  vector<Particle> resampled_particles;
  for(int i = 0; i < num_particles; i++) 
  {
    beta += weight_dist(gen) * 2.0;
    while( beta > weights[index]) 
    {
      beta -= weights[index];
      index = (index + 1) % num_particles;
    }
    resampled_particles.push_back(particles[index]);
  }

  particles = resampled_particles;
}

  
void ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
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
