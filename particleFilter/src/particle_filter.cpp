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

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
    // TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
    //   x, y, theta and their uncertainties from GPS) and all weights to 1.
    // Add random Gaussian noise to each particle.
    // NOTE: Consult particle_filter.h for more information about this method (and others in this file).
    //don't like the hard coding but I need to follow the structure I guess
    num_particles = 50;
    double weight = 1.0;
    // noise generation
    default_random_engine gen;
    normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);

    for (int i = 0; i < num_particles; i++) {
        Particle particle = {i, dist_x(gen), dist_y(gen), dist_theta(gen), weight};
        particles.push_back(particle);
        weights.push_back(weight);
    }
    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
    // TODO: Add measurements to each particle and add random Gaussian noise.
    // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
    //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
    //  http://www.cplusplus.com/reference/random/default_random_engine/
    default_random_engine gen;
    for (int i = 0; i < num_particles; i++) {
        double new_theta = particles[i].theta + yaw_rate * delta_t;
        double new_x, new_y;
        if (abs(yaw_rate) > 0.00001) {
            new_x = particles[i].x + velocity / yaw_rate * (sin(new_theta) - sin(particles[i].theta));
            new_y = particles[i].y + velocity / yaw_rate * (cos(particles[i].theta) - cos(new_theta));
        } else {
            new_x = particles[i].x + velocity * delta_t * cos(particles[i].theta);
            new_y = particles[i].y + velocity * delta_t * sin(particles[i].theta);
        }
        normal_distribution<double> dist_x(new_x, std_pos[0]);
        normal_distribution<double> dist_y(new_y, std_pos[1]);
        normal_distribution<double> dist_theta(new_theta, std_pos[2]);
        particles[i].x = dist_x(gen);
        particles[i].y = dist_y(gen);
        particles[i].theta = dist_theta(gen);
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

    for (int i = 0; i < num_particles; i++) {

        long double weight = 1;

        for (int j = 0; j < observations.size(); j++) {
            //given a particle and an observation, transform the observation from vehicle's coordinate system
            //into map's coordinate system
            double map_observation_x = 0;
            double map_observation_y = 0;
            double cos_theta = cos(particles[i].theta);
            double sin_theta = sin(particles[i].theta);

            map_observation_x = observations[j].x * cos_theta - observations[j].y * sin_theta + particles[i].x;
            map_observation_y = observations[j].y * cos_theta + observations[j].x * sin_theta + particles[i].y;

            Map::single_landmark_s nearest_landmark;
            //given an observation, find the nearest landmark
            double distance = 0;
            double smallest_distance = sensor_range;
            for (int k = 0; k < map_landmarks.landmark_list.size(); k++) {

                Map::single_landmark_s landmark = map_landmarks.landmark_list[k];

                distance = fabs(map_observation_x - landmark.x_f) + fabs(map_observation_y - landmark.y_f);

                if (distance < smallest_distance) {
                    smallest_distance = distance;
                    nearest_landmark = landmark;
                }
            }

            // calculate weight multiplier and updates weights
            double x_diff = nearest_landmark.x_f - map_observation_x;
            double y_diff = nearest_landmark.y_f - map_observation_y;
            double std_x = std_landmark[0];
            double std_y = std_landmark[1];

            long double nominator = exp(-0.5 * ((x_diff * x_diff) / ( std_x * std_x) + (y_diff * y_diff) / (std_y * std_y)));
            long double denominator = 2 * M_PI * std_x * std_y;
            long double weight_multiplier = nominator / denominator;

            weight *= weight_multiplier;
        }

        particles[i].weight = weight;
        weights[i] = weight;
    }
}

void ParticleFilter::resample() {
    // TODO: Resample particles with replacement with probability proportional to their weight.
    // NOTE: You may find std::discrete_distribution helpful here.
    //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

    default_random_engine gen;
    discrete_distribution<> distribution(weights.begin(), weights.end());
    vector<Particle> resampled;
    for (int i = 0; i < num_particles; ++i){
        resampled.push_back(particles[distribution(gen)]);
    }
    particles = resampled;

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
