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

#include "particle_filter.h"
#include "helper_functions.h"

inline double normalize_angle(double angle) {
  return fmod(angle, 2.0 * M_PI);
}

std::string particle_str(Particle p) {
  std::ostringstream pout;
  pout << "P:[" << p.id << ", " << p.x << ", " << p.y << ", "
       << p.theta << ", " << p.weight << "]";
  return pout.str();
}


void print_particles(std::vector<Particle> particles) {
  int num_particles = particles.size();
  for (int i = 0; i < num_particles; ++i) {
    std::cout << particle_str(particles[i]) << std::endl;
  }
}

//  void transformObservations(Particle particle, std::vector<LandmarkObs>& observations);
//  void predictObservations(Particle particle, double sensor_range, Map map_landmarks, std::vector<LandmarkObs>& predicted);
;


void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

//	std::cout << "Particle::init: x=" << x << ", " << "y=" << y << ", "
//						<< "theta=" << theta << std::endl;

	num_particles = 30; //100 - it works even with 10 particles
  particles.clear();
  particles.reserve(num_particles);
  weights.clear();
  weights.reserve(num_particles);

	// noise generation
	std::default_random_engine gen;
	std::normal_distribution<double> N_x(0, std[0]);
	std::normal_distribution<double> N_y(0, std[1]);
	std::normal_distribution<double> N_theta(0, std[2]);

	for (int i = 0; i < num_particles; ++i) {
    Particle p;
    p.id = i;
    p.x = x + N_x(gen);
    p.y = y + N_y(gen);
    p.theta = normalize_angle(theta + N_theta(gen));
    p.weight = 1;
    particles.push_back(p);
    weights.push_back(p.weight);

	}

  is_initialized = true;


}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

  // noise generation
  std::default_random_engine gen;
  std::normal_distribution<double> N_x(0, std_pos[0]);
  std::normal_distribution<double> N_y(0, std_pos[1]);
  std::normal_distribution<double> N_theta(0, std_pos[2]);

  if (fabs(yaw_rate) < 1.0e-06) {
    // yaw rate is "ZERO"
//    std::cout << "YAW RATE is ZERO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    for (int i = 0; i < num_particles; ++i) {

      particles[i].x += velocity * cos(particles[i].theta) * delta_t;
      particles[i].y += velocity * (sin(particles[i].theta) * delta_t);

      //add noise
      particles[i].x += N_x(gen);
      particles[i].y += N_y(gen);
      particles[i].theta = normalize_angle(particles[i].theta + N_theta(gen));

      particles[i].id = i;
    }
  } else {
    for (int i = 0; i < num_particles; ++i) {
      particles[i].x += (velocity / yaw_rate) * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
      particles[i].y += (velocity / yaw_rate) * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
      particles[i].theta += yaw_rate * delta_t;

      //add noise
      particles[i].x += N_x(gen);
      particles[i].y += N_y(gen);
      particles[i].theta = normalize_angle(particles[i].theta + N_theta(gen));

      particles[i].id = i;
    }
  }


}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

  // implemented in ParticleFilter::updateWeights method

}

//void ParticleFilter::transformObservations(Particle particle, std::vector<LandmarkObs>& observations) {
void transformObservations(Particle particle, std::vector<LandmarkObs>& observations) {
  // transforms observations from Vehicle's coordinates to Map's for given particle

  int obs_size = observations.size();
  for (int i = 0; i < obs_size; ++i) {
    double x_d = cos(particle.theta) * observations[i].x - sin(particle.theta) * observations[i].y + particle.x;
    double y_d = sin(particle.theta) * observations[i].x + cos(particle.theta) * observations[i].y + particle.y;
    observations[i].x = x_d;
    observations[i].y = y_d;

    // MAGIC: this is a mystery of C++ for some reason the next 2 lines of code is not calculating observations[i].y correctly
    // I tried casting but is didn't help. But the above code with an itermediary var y_d works well :)
//    observations[i].x = cos(particle.theta) * observations[i].x - sin(particle.theta) * observations[i].y + particle.x;
//    observations[i].y = sin(particle.theta) * observations[i].x + cos(particle.theta) * observations[i].y + particle.y;

  }
}

//void ParticleFilter::predictObservations(Particle particle, double sensor_range, Map map_landmarks, std::vector<LandmarkObs>& predicted) {
void predictObservations(Particle particle, double sensor_range, Map map_landmarks, std::vector<LandmarkObs>& predicted) {
  int map_size = map_landmarks.landmark_list.size();
  for (int i = 0; i < map_size; ++i) {
    double d = dist(particle.x, particle.y, map_landmarks.landmark_list[i].x_f, map_landmarks.landmark_list[i].y_f);
    if (d < sensor_range) {
      LandmarkObs obs;
      obs.x = map_landmarks.landmark_list[i].x_f;
      obs.y = map_landmarks.landmark_list[i].y_f;
      obs.id = map_landmarks.landmark_list[i].id_i;
      predicted.push_back(obs);

    }

  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account 
	//   for the fact that the map's y-axis actually points downwards.)
	//   http://planning.cs.uiuc.edu/node99.html

  double weight_sum = 0.0;

  for (int i = 0; i < num_particles; ++i) {
    Particle p = particles[i];

    std::vector<LandmarkObs> transObservations(observations);
    transformObservations(p, transObservations);

    std::vector<LandmarkObs> predicted;
    predictObservations(p, sensor_range, map_landmarks, predicted);

//    std::cout << "Transformed observations: for " << particle_str(p) << std::endl;
//    print_observations(transObservations);
//    std::cout << "Predicted Observations: " << std::endl;
//    print_observations(predicted);

    double weight = 1.0;

    int obs_size = transObservations.size();
    for (int j = 0; j < obs_size; ++j) {

      // Find closest landmark
      double min_d = sensor_range;
      LandmarkObs nearest_landmark;
      bool found = false;
      int pred_size = predicted.size();
      for (int k = 0; k < pred_size; ++k) {
        double d = dist(transObservations[j].x, transObservations[j].y, predicted[k].x, predicted[k].y);
        if (d < min_d) {
          nearest_landmark = predicted[k];
          min_d = d;
          found = true;
        }
      }


      // Update particle weight
      double exX, exY;

      if (found) {
        exX = (transObservations[j].x - nearest_landmark.x);
        exY = (transObservations[j].y - nearest_landmark.y);
      } else {
        exX = sensor_range;
        exY = sensor_range;
      }

      exX = exX * exX;
      exX /= 2 * std_landmark[0] * std_landmark[0];

      exY = exY * exY;
      exY /= 2 * std_landmark[1] * std_landmark[1];

      double ex = -1.0 * (exX + exY);

      double prob = (1.0 / sqrt(2 * M_PI * std_landmark[0] * std_landmark[1]) ) * exp(ex);

      weight *= prob;

//      std::cout << "prob = " << prob << " O=" << obs_str(transObservations[j]) << " P=" << obs_str(nearest_landmark) << ": " << nearest_landmark.id << " min_d = " << min_d << std::endl;


    }

//    std::cout << "[" << i << "] weight = " << weight << " for " << particle_str(p) << std::endl;

    weight_sum += weight;

    p.weight = weight;
    weights[i] = weight;

    particles[i] = p;


//    std::cin.ignore();
  }


}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

  // Don't need normalization because of std::discrete_distribution
//  double weight_sum = 0.0;
//  for (int i = 0; i < num_particles; ++i) {
//    weight_sum += weights[i];
//  }
//
//  // Normalize weights
//  for (int i = 0; i < num_particles; ++i) {
//    if (weight_sum == 0.0) {
//      particles[i].weight = 1.0 / num_particles;
//      weights[i] = 1.0 / num_particles;
//    } else {
//      particles[i].weight /= weight_sum;
//      weights[i] /= weight_sum;
//    }
//  }

  std::vector<Particle> new_particles;
  new_particles.reserve(num_particles);


  std::default_random_engine gen;
  std::discrete_distribution<> d(weights.begin(), weights.end());


  for (int i = 0; i < num_particles; ++i) {
    new_particles[i] = particles[d(gen)];
  }

  // MAGIC: (need tweak C++ more for this)
  // Strange: operator "=" or assign is not working for vector
  // particles.assign(new_particles.begin(), new_particles.end());
  // or particles = new_particles
  // Why?
  for (int i = 0; i < num_particles; ++i) {
    particles[i] = new_particles[i];
  }



}

void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}
