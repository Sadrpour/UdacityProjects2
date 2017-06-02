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
    num_particles =500;
    default_random_engine gen;
    normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);
    particles.resize(num_particles);
    for(int i=0;i<num_particles;++i){
        particles[i].id = i;
        particles[i].x = dist_x(gen);
        particles[i].y = dist_y(gen);
        particles[i].theta = dist_theta(gen);
        particles[i].weight = 1.0;
    }
      // Initialize all weights to 1.
    weights.clear();
    weights.resize(num_particles);
    for (int i = 0; i < weights.size(); i++) {
    weights[i] = 1.0;
    }

    is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
    default_random_engine gen;
    normal_distribution<double> dist_x(0, std_pos[0]);
	normal_distribution<double> dist_y(0, std_pos[1]);
	normal_distribution<double> dist_theta(0, std_pos[2]);
	float sx = 0;
	float sy = 0;
	float st = 0;
    for(int i=0;i<num_particles;i++){
        Particle P = particles[i];
        if(fabs(yaw_rate)>1E-5){
        P.x += velocity/yaw_rate*(sin(P.theta+yaw_rate*delta_t)-sin(P.theta)) + dist_x(gen);
        P.y += velocity/yaw_rate*(cos(P.theta)-cos(P.theta+yaw_rate*delta_t)) + dist_y(gen);
        P.theta += delta_t*yaw_rate + dist_theta(gen);
        } else {
        P.x += velocity*delta_t*(cos(P.theta)) + dist_x(gen);  // change sin and cos here x cos , y sin
        P.y += velocity*delta_t*(sin(P.theta)) + dist_y(gen);
        P.theta += dist_theta(gen);
        }
        if (i==0){}
        particles[i].x = P.x;
        particles[i].y = P.y;
        particles[i].theta = P.theta;
        sx += P.x;
        sy += P.y;
        st += P.theta;

    }
//  cout << "average x" << sx/num_particles << "average y" << sy/num_particles << "average theta" << st/num_particles << endl;

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
    float sx = std_landmark[0];
    float sy = std_landmark[1];
    float s = 0;
    for(int p=0;p<num_particles;p++){
        particles[p].weight = 1.0;
        for(int o=0;o<observations.size();o++){
            float prediction_x =  observations[o].x*cos(particles[p].theta) - observations[o].y*sin(particles[p].theta) + particles[p].x;
            float prediction_y =  observations[o].x*sin(particles[p].theta) + observations[o].y*cos(particles[p].theta) + particles[p].y;
            float minDistance = 1000;
            int inde = 0;
            float x_diff = 1000;
            float y_diff = 1000;
            float d = -1;
//            cout << "update weights in PF 1.3" << endl;
            for(int l=0;l<map_landmarks.landmark_list.size();l++){
                float landx = map_landmarks.landmark_list[l].x_f;
                float landy = map_landmarks.landmark_list[l].y_f;
                if (dist(landx,landy,particles[p].x,particles[p].y) < 50){
                float d = dist(prediction_x, prediction_y, landx,landy);
//                cout << "distance to nearby landmarks" << d << endl;
                if (d < minDistance){
                    minDistance = d;
                    x_diff = prediction_x - landx;
                    y_diff = prediction_y - landy;
                    inde = l;
                }}}
            observations[o].id = inde;
//            cout << "for particle" << p <<"distance to closest land mark is " <<  d  << "for landmark id " << inde << " and observation id "<< o << endl;
            particles[p].weight *= ( 1/(2*M_PI*sx*sy)) * exp( -1*( x_diff*x_diff/(2*sx*sx) +  y_diff*y_diff/(2*sy*sy) )) ;
            }
    if(particles[p].weight < 1E-6){
        particles[p].weight = 1E-6;
    }
    s += particles[p].weight;
    }

    weights.clear();
    for(int p=0;p<num_particles;p++){ // normalizing the particle weights
    particles[p].weight = particles[p].weight/s;
    weights.push_back(particles[p].weight);
    }



}

void ParticleFilter::resample() {

  std::vector<Particle> Resampledparticles;
  std::discrete_distribution<> densityVector(weights.begin(), weights.end());
  default_random_engine gen;

 for (int p = 0; p<num_particles; p++) {
  int particleIndex = densityVector(gen);
  Resampledparticles.push_back(particles[particleIndex]);
 }

  particles = Resampledparticles;


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
