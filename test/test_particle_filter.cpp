//
// Created by Andrii Garkavyi on 12/8/17.
//

#include "gtest/gtest.h"
#include "particle_filter.h"

using namespace std;

double SIGMA_POS[3] = {0.3, 0.3, 0.01};
const int NUM_POINTS = 100;


class ParticleFilterTestSuite : public ::testing::Test {
protected:
  ParticleFilter pf;

  virtual void SetUp() {
    //cout << "Setup call " << endl;
    pf.init(10.0, 10.0, M_PI/4, SIGMA_POS);
  }

  /**
   * Calculate average X and Y for all particles in ParticleFilter object.
   * Average value later can be used to test against expected value.
   * @param x variable to pass X value
   * @param y variable to pass Y value
   */
  void getAverageParticleXY(double& x, double& y) {
    long num_particles = pf.particles.size();
    cout<< "Particles amount: " << num_particles << endl;

    double x_sum = 0.0;
    double y_sum = 0.0;
    for (int i = 0; i < num_particles; ++i) {
//    printParticle(particles[i]);
      x_sum += pf.particles[i].x;
      y_sum += pf.particles[i].y;
    }
    x_sum /= num_particles;
    y_sum /= num_particles;

    x = x_sum;
    y = y_sum;
  }

};


TEST_F(ParticleFilterTestSuite, TestInit) {

  ASSERT_EQ(pf.particles.size(), NUM_POINTS);

//  for (int i=0; i < num; i++) {
//    printParticle(pf.particles[i]);
//  }

}

TEST_F(ParticleFilterTestSuite, TestPredictSamePlace) {
  pf.prediction(0.1, SIGMA_POS, 0, 0);

  double x_avg;
  double y_avg;
  getAverageParticleXY(x_avg, y_avg);

  ASSERT_NEAR(x_avg, 10.0, .15) << "X values should be within 10.0";
  ASSERT_NEAR(y_avg, 10.0, .15) << "Y values should be within 10.0";
}

TEST_F(ParticleFilterTestSuite, TestPredictSlowMove) {
  pf.prediction(0.1, SIGMA_POS, 10.0, 0); // velocity=10m/s or 1m per 0.1sec - should go diagonal

  double x_avg;
  double y_avg;
  getAverageParticleXY(x_avg, y_avg);

  ASSERT_NEAR(x_avg, 10.707, .15) << "X values should be within 10.0 + sqrt(2)/2";
  ASSERT_NEAR(y_avg, 10.707, .15) << "Y values should be within 10.0 + sqrt(2)/2";
}

TEST_F(ParticleFilterTestSuite, TestPredictSlowMoveAndTurn) {
  // velocity=10m/s or 1m per 0.1sec, rotation PI/4 per 0.1 sec. Should be almost straight up.
  pf.prediction(0.1, SIGMA_POS, 20.0, 10*M_PI/4);

  double x_avg;
  double y_avg;
  getAverageParticleXY(x_avg, y_avg);

  ASSERT_NEAR(x_avg, 10.7, .15);
  ASSERT_NEAR(y_avg, 11.7, .15);
}

TEST_F(ParticleFilterTestSuite, TestPredictSlowMoveAndNegativeTurn) {
  // velocity=10m/s or 1m per 0.1sec, rotation PI/4 per 0.1 sec. Should be almost straight up.
  pf.prediction(0.1, SIGMA_POS, 20.0, -10*M_PI/4);

  double x_avg;
  double y_avg;
  getAverageParticleXY(x_avg, y_avg);

  ASSERT_NEAR(x_avg, 11.8, .15);
  ASSERT_NEAR(y_avg, 10.7, .15);
}


// ============= Test coordinates translation from car to map reference space ===============
TEST_F(ParticleFilterTestSuite, TestTranslateCoords) {

  Particle p;
  p.x = 10.0;
  p.y = 10.0;
  p.theta = M_PI/2;

  vector<LandmarkObs> observations;
  LandmarkObs obs1;
  obs1.x = 1.0;
  obs1.y = 1.0;
  observations.push_back(obs1);

  vector<LandmarkObs> obs_map = pf.translateCoordinatesFromParticleToMap(p, observations);

  ASSERT_EQ(obs_map.size(), 1);
  ASSERT_FLOAT_EQ(obs_map[0].x, 9.0);
  ASSERT_FLOAT_EQ(obs_map[0].y, 11.0);

}

TEST_F(ParticleFilterTestSuite, TestTranslateCoordsZero) {

  Particle p;
  p.x = 10.0;
  p.y = 10.0;
  p.theta = M_PI/2;

  vector<LandmarkObs> observations;
  LandmarkObs obs1;
  obs1.x = 0.0;
  obs1.y = 0.0;
  observations.push_back(obs1);

  vector<LandmarkObs> obs_map = pf.translateCoordinatesFromParticleToMap(p, observations);

  ASSERT_EQ(obs_map.size(), 1);
  ASSERT_FLOAT_EQ(obs_map[0].x, 10.0);
  ASSERT_FLOAT_EQ(obs_map[0].y, 10.0);

}

TEST_F(ParticleFilterTestSuite, TestTranslateCoordsTwo) {

  Particle p;
  p.x = 10.0;
  p.y = 10.0;
  p.theta = M_PI/2;

  vector<LandmarkObs> observations;
  LandmarkObs obs1;
  obs1.x = 1.0;
  obs1.y = 1.0;
  observations.push_back(obs1);

  LandmarkObs obs2;
  obs1.x = 1.0;
  obs1.y = -1.0;
  observations.push_back(obs1);

  vector<LandmarkObs> obs_map = pf.translateCoordinatesFromParticleToMap(p, observations);

  ASSERT_EQ(obs_map.size(), 2);
  ASSERT_FLOAT_EQ(obs_map[1].x, 11.0);
  ASSERT_FLOAT_EQ(obs_map[1].y, 11.0);

}

TEST_F(ParticleFilterTestSuite, TestResampleBasic) {
  ParticleFilter filter;
  double std[3] = {0.2, 0.2, 0.1};
  filter.init(0.1, 0.1, 0.1, std); // num initialized to 100

  filter.particles.clear(); //we want an experiment
  filter.particles.push_back({0, 1.0, 1.0, 0.1, 0.9});
  filter.particles.push_back({2, 2.0, 2.0, 0.2, 0.1});

  filter.resample();

  ASSERT_EQ(filter.particles.size(), NUM_POINTS);

  //count particles
  int ctr1 = 0;
  int ctr2 = 0;
  for (int i=0; i<NUM_POINTS; i++) {
//    printParticle(filter.particles[i]);
    if (filter.particles[i].id == 0) { ctr1++; }
    else { ctr2++; }
  }

  ASSERT_GT(ctr1, NUM_POINTS*0.8);
  ASSERT_LT(ctr2, NUM_POINTS*0.15);
}
