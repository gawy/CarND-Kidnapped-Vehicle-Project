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

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

  num_particles = 100;
  normal_distribution<double> distribution_x(x, std[0]);
  normal_distribution<double> distribution_y(y, std[1]);
  normal_distribution<double> distribution_theta(theta, std[2]);
  std::default_random_engine gen;

  for (int i=0; i < num_particles; i++) {
    Particle p;
    p.id = i;
    p.x = distribution_x(gen);
    p.y = distribution_y(gen);
    p.theta = distribution_theta(gen);
    p.weight = 1.0;

    particles.push_back(p);
  }

  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

  std::random_device rd{};
  std::mt19937 gen{rd()};

  cout << "Predicting for velocity=" << velocity <<"; yaw_rate="<<yaw_rate << endl;

  for (int i=0; i < num_particles; i++) {
    Particle& p = particles[i];

    if (abs(yaw_rate) > 0) { // yaw rate not equal to zero
      p.x = p.x + velocity/yaw_rate * (sin(p.theta + yaw_rate*delta_t) - sin(p.theta));
      p.y = p.y + velocity/yaw_rate * (cos(p.theta) - cos(p.theta + yaw_rate*delta_t));
      p.theta += yaw_rate * delta_t;

      // yaw is modified only in this case so we add noise only here
      normal_distribution<double> distribution_theta(p.theta, std_pos[2]);
      p.theta = distribution_theta(gen);
    } else { // yaw rate equal to zero
//      cout << p.x << " => " << p.x + velocity * delta_t * cos(p.theta) << endl;
      p.x = p.x + velocity * delta_t * cos(p.theta);
      p.y = p.y + velocity * delta_t * sin(p.theta);
      // theta does not change
    }

    // add noise to positional variables
    normal_distribution<double> distribution_x(p.x, std_pos[0]);
    normal_distribution<double> distribution_y(p.y, std_pos[1]);
    p.x = distribution_x(gen);
    p.y = distribution_y(gen);

//    cout << "Predicted particle: ";
//    printParticle(particles[i]);
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html



  // translate coordinates from car frame of reference to map
  for (int i=0; i < num_particles; i++) {
    Particle& p = particles[i];

    // convert observations' coordinates to map coord. system
    std::vector<LandmarkObs> obs_map = translateCoordinatesFromParticleToMap(p, observations);

    // associate observations of particle with landmark positions
    vector<Map::single_landmark_s> landmarks = map_landmarks.landmarksInRange(sensor_range, p.x, p.y);
    associateObservationsWithLandmarks(p, obs_map, landmarks, std_landmark);


//    cout << "Weight update particle: ";
//    printParticle(particles[i]);
  }
}

void ParticleFilter::resample() {
	// Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

  vector<double> weights;
  for (int i=0; i<particles.size(); i++) {
    weights.push_back(particles[i].weight);
  }

  discrete_distribution<> distribution(weights.begin(), weights.end());
  std::random_device rd{};
  std::mt19937 gen{rd()};

  vector<Particle> p_new;
  for (int i=0; i<num_particles; i++) {
    Particle p = particles[distribution(gen)];
    p_new.push_back(p);

//    cout << "Resample: ";
//    printParticle(p);
  }
  particles.clear();
  particles = move(p_new);

//  cout << "End cycle" << endl;
//  cout << "Predict: num="<< particles.size() << ", "
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

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

std::vector<LandmarkObs>
ParticleFilter::translateCoordinatesFromParticleToMap(Particle &particle, const std::vector<LandmarkObs> &observations) {
  vector<LandmarkObs> obs_map;

//  cout << "Transforming coordinates for particle: ";
//  printParticle(particle);

  for (int i=0; i < observations.size(); i++) {
//    observations[i]
    LandmarkObs obs;
    obs.x = observations[i].x * cos(particle.theta) - observations[i].y * sin(particle.theta) + particle.x;
    obs.y = observations[i].x * sin(particle.theta) + observations[i].y * cos(particle.theta) + particle.y;
    obs.id = observations[i].id;

    obs_map.push_back(obs);

//    cout << "Transform: ("<<observations[i].x <<","<<observations[i].y<<")=>("<<obs.x <<","<< obs.y <<")" << endl;
  }

  return obs_map;
}

void ParticleFilter::associateObservationsWithLandmarks(Particle &particle,
                                                       std::vector<LandmarkObs> &observations,
                                                       const std::vector<Map::single_landmark_s> &map_landmarks,
                                                       double std_landmark[]) {
  if (map_landmarks.size() == 0 || observations.size() == 0) {
    return;
  }

  particle.associations.clear();
  particle.sense_x.clear();
  particle.sense_y.clear();

  double weight = 1.0;
  double sig_x = std_landmark[0];
  double sig_y = std_landmark[1];

  for (int io=0; io<observations.size(); io++) {
    LandmarkObs& obs = observations[io];

    // find landmark that is closest to the observation
    const Map::single_landmark_s& mark = map_landmarks[0];
    double dist_min = sqrt((obs.x - mark.x_f)*(obs.x - mark.x_f) + (obs.y - mark.y_f)*(obs.y - mark.y_f));
    int idx_min = mark.id_i;
    int i_min = 0;
//    cout<< "dist=" << dist_min << endl;

    for (int im=1; im<map_landmarks.size(); im++) {
      const Map::single_landmark_s& m = map_landmarks[im];
      double dist = sqrt((obs.x - m.x_f)*(obs.x - m.x_f) + (obs.y - m.y_f)*(obs.y - m.y_f));

//      cout << im << " dist=" << dist << endl;
      if (dist < dist_min) {
        dist_min = dist;
        idx_min = m.id_i;
        i_min = im;
      }
    }

    particle.associations.push_back(idx_min); // save landmark index
    particle.sense_x.push_back(obs.x);
    particle.sense_y.push_back(obs.y);

    //calculate weight of particle based on its observations
    const Map::single_landmark_s& landmark = map_landmarks[i_min];
    double w1 = (landmark.x_f - obs.x)*(landmark.x_f - obs.x)/(2*sig_x*sig_x)
             + (landmark.y_f - obs.y)*(landmark.y_f - obs.y)/(2*sig_y*sig_y);
    double w2 = 1 / (2*M_PI*sig_x*sig_y)
              * exp(-( w1 ));

    weight *= w2;
//    cout << obs.x <<","<<obs.y << " associated with " << landmark.id_i << "(" << landmark.x_f<<","<< landmark.y_f <<") w="<< w2 <<" total weight: "<< weight<< endl;
  }

  particle.weight = weight;
}

void printParticle(Particle &p) {
  cout<< "<" <<p.id<< "> " << p.x <<", "<< p.y << ", " << p.theta << ", " << p.weight << endl;
}

void printObservation(LandmarkObs &o) {
  cout<< "<" <<o.id<< "> " << o.x <<", "<< o.y << endl;
}
