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
      	min_dist = 1000000.0;
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


void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], const std::vector<LandmarkObs> &observations,
			const Map &map_landmarks)
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
  double std_landmark_range = std_landmark[0];
  double std_landmark_bearing = std_landmark[1];

  for (int p_index = 0; p_index < num_particles; p_index++) 
  {
    double particle_x = particles[p_index].x;
    double particle_y = particles[p_index].y;
    double particle_theta = particles[p_index].theta;
  
    // Find landmarks in particle's range.
    vector<LandmarkObs> in_range_landmarks;
    LandmarkObs in_range_landmark;
    for(unsigned int index = 0; index < map_landmarks.landmark_list.size(); index++) 
    {
      in_range_landmark.x = map_landmarks.landmark_list[index].x_f;
      in_range_landmark.y = map_landmarks.landmark_list[index].y_f;
      in_range_landmark.id = map_landmarks.landmark_list[index].id_i;
      double dx = particle_x - in_range_landmark.x;
      double dy = particle_y - in_range_landmark.y;
      double distance = sqrt(dx * dx + dy * dy);
      if ( distance <= sensor_range ) 
      {
        in_range_landmarks.push_back(in_range_landmark);
      }
    }

    // Transform observation coordinates.
    vector<LandmarkObs> trans_observations;
    LandmarkObs trans_obs;
    for(unsigned int obs_index = 0; obs_index < observations.size(); obs_index++) 
    {
      LandmarkObs obs = observations[obs_index];
      trans_obs.x = cos(particle_theta)*obs.x - sin(particle_theta)*obs.y + particle_x;
      trans_obs.y = sin(particle_theta)*obs.x + cos(particle_theta)*obs.y + particle_y;
      trans_obs.id = obs.id;
      trans_observations.push_back(trans_obs);
    }

    // Observation association to landmark.
    dataAssociation(in_range_landmarks, trans_observations);

    // Reseting weight.
    particles[p_index].weight = 1.0;

    // Calculate weights.
    LandmarkObs trans_observation;
    LandmarkObs landmark;
    for(unsigned int trans_obs_indx = 0; trans_obs_indx < trans_observations.size(); trans_obs_indx++) 
    {
      trans_observation.x = trans_observations[trans_obs_indx].x;
      trans_observation.y = trans_observations[trans_obs_indx].y;
      trans_observation.id = trans_observations[trans_obs_indx].id;

      unsigned int k = 0;
      bool found = false;
      while( !found && k < in_range_landmarks.size() ) 
      {
        if ( in_range_landmarks[k].id == trans_observation.id) 
        {
          found = true;
          landmark.x = in_range_landmarks[k].x;
          landmark.y = in_range_landmarks[k].y;
        }
        k++;
      }

      // Calculating weight.
      double dX = trans_observation.x - landmark.x;
      double dY = trans_observation.y - landmark.y;
      double const_1= ( 1/(2*M_PI*std_landmark_range*std_landmark_bearing));
      double const_2 =  -( dX*dX/(2*std_landmark_range*std_landmark_range) + (dY*dY/(2*std_landmark_bearing*std_landmark_bearing)) );
      double weight = const_1 * exp(const_2);
      if (0 == weight) 
      {
        particles[p_index].weight *= 0.00001;
      } 
      else 
      {
        particles[p_index].weight *= weight;
      }
    }
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
