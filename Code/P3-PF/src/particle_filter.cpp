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
//pf.init(sense_x, sense_y, sense_theta, sigma_pos)
void ParticleFilter::init(double x, double y, double theta, double std[]) {
    num_particles = 100;
    cout << num_particles << endl;
    default_random_engine gen;
    normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);
    vector<Particle> particles;
    particles.resize(num_particles);
    for(int i=0;i<num_particles;++i){
        cout << "particle created: "<< i << endl;
        Particle P;
        P.id = i; // wondering why this does not work ?!?!
        particles[i].id = i;
        //P.x = dist_x(gen);  // wondering why this does not work ?!?!
        particles[i].x = dist_x(gen);
        //P.y = dist_y(gen); // wondering why this does not work ?!?!
        particles[i].y = dist_y(gen);
        //P.theta = dist_theta(gen); // wondering why this does not work ?!?!
        particles[i].theta = dist_theta(gen);
        //P.weight = 1.0; // wondering why this does not work ?!?!
        particles[i].weight = 1.0;
        // particles.push_back(P);
    }
    is_initialized = true;
    cout << "weight of the first particle: " << particles[10].x << endl;

	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
    cout << "prediction in PF" << endl;
    default_random_engine gen;
    normal_distribution<double> dist_x(0, std_pos[0]);
	normal_distribution<double> dist_y(0, std_pos[1]);
	normal_distribution<double> dist_theta(0, std_pos[2]);
	cout << "particle size : "<< num_particles << endl;
    for(int i=0;i<num_particles;++i){
        Particle P = particles[i];
        if(yaw_rate>0.01){
        P.x = P.x + velocity/yaw_rate*(sin(P.theta+yaw_rate*delta_t)-sin(P.theta)) + dist_x(gen);
        P.y = P.x + velocity/yaw_rate*(cos(P.theta)-cos(P.theta+yaw_rate*delta_t)) + dist_y(gen);
        P.theta = P.theta + delta_t*yaw_rate + dist_theta(gen);
        } else {
        P.x = P.x + velocity*delta_t*(cos(P.theta)) + dist_x(gen);
        P.y = P.y + velocity*delta_t*(sin(P.theta)) + dist_y(gen);
        P.theta = P.theta + dist_theta(gen);
        }
        cout << "particle prediction : "<< i << endl;
        if (i=0){
          cout << "particle prediction : "<< P.x << "," << P.y << "," << P.theta << endl;
        }
        particles[i].x = P.x;
        particles[i].y = P.y;
        particles[i].theta = P.theta;
    }
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
    //cout << "highest w " << highest_weight << endl;
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
    std::vector<LandmarkObs> observations, Map map_landmarks) {
    cout << "update weights in PF" << endl;
    float sx = std_landmark[0];
    float sy = std_landmark[1];
    float s = 0;
    cout << "update weights in PF 1.1" << endl;
    for(int p=0;p<num_particles;++p){
        cout << "update weights in PF 1.0.1" << endl;
        cout << "iteration " <<  p << endl;
        cout << "x location " <<  particles[10].x << endl;
        cout << "particle weight" << particles[p].weight  << endl;
        particles[p].weight = 1.0;
        cout << "update weights in PF 1.0.2" << endl;
        for(int o=0;o<observations.size();++o){
                // observations from local to global frame
                cout << "update weights in PF 1.2" << endl;
                float prediction_x =  observations[o].x*cos(particles[p].theta) - observations[o].y*sin(particles[p].theta) + particles[p].x;
                float prediction_y =  observations[o].x*sin(particles[p].theta) + observations[o].y*cos(particles[p].theta) + particles[p].y;
            float m = 1000;
            int inde = 0;
            float x_diff = 0;
            float y_diff = 0;
            cout << "update weights in PF 1.3" << endl;
            for(int l=0;l<map_landmarks.landmark_list.size();++l){
                float x = map_landmarks.landmark_list[l].x_f;
                float y = map_landmarks.landmark_list[l].y_f;
                float d = dist(prediction_x, prediction_y, x,y);
                if (d < m){
                    m = d;
                    inde = l;
                    x_diff = prediction_x - x;
                    y_diff = prediction_y - y;
                }
            if (d >=0 ){// there was an assignment
            particles[p].weight *= ( 1/(2*M_PI*sx*sy)) * exp( -( x_diff*x_diff/(2*sx*sx) +  y_diff*y_diff/(2*sy*sy)  )) ;
            }

            }
        }
    cout << "update weights in PF 2.1" << endl;
    s += particles[p].weight;
    }
    cout << "update weights in PF 3.1" << endl;
    for(int p=0;p<num_particles;++p){ // normalizing the particle weights
    particles[p].weight = particles[p].weight/s;
    }

}

void ParticleFilter::resample() {
  cout << "resample in PF" << endl;
  cout << "resample in PF 0.1" << endl;
  vector<Particle> Resampledparticles;
  cout << "resample in PF 1.0" << endl;

  std::default_random_engine generator;
  cout << "resample in PF 1.1" << endl;
  std::uniform_real_distribution<double> distribution(0.0,1.0);
  cout << "resample in PF 1.2" << endl;
  int ind = int(distribution(generator)*num_particles);
  cout << "resample in PF 1.3" << endl;
  float beta = 0.0;
  float mw = 0.0;
  cout << "resample in PF 1" << endl;
  for(int p=0;p<num_particles;++p){ // find maximum weight
    if (particles[p].weight > mw){
       mw = particles[p].weight;
     }
    }

  for (int i=0; i<num_particles; ++i) {
    beta += distribution(generator)*2.0*mw;
    while(beta > particles[ind].weight){
     beta += -1*particles[ind].weight;
     ind = (ind + 1)%particles.size();
    }
    Resampledparticles.push_back(particles[ind]);
  }
  cout << "resample in PF 2" << endl;
  particles = Resampledparticles;

	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

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
