//
// Created by Andrii Garkavyi on 12/8/17.
//

#include "gtest/gtest.h"
#include "particle_filter.h"

using namespace std;

double SIGMA_POS[3] = {0.3, 0.3, 0.01};


class ParticleFilterTestSuite : public ::testing::Test {
protected:
  ParticleFilter pf;

  virtual void SetUp() {
    cout << "Setup call " << endl;
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
  const int num = 100;

  ASSERT_EQ(pf.particles.size(), num);

  for (int i=0; i < num; i++) {
    printParticle(pf.particles[i]);
  }

}

TEST_F(ParticleFilterTestSuite, TestPredictSamePlace) {
  pf.prediction(0.1, SIGMA_POS, 0, 0);

  double x_avg;
  double y_avg;
  getAverageParticleXY(x_avg, y_avg);

  ASSERT_NEAR(x_avg, 10.0, .1) << "X values should be within 10.0";
  ASSERT_NEAR(y_avg, 10.0, .1) << "Y values should be within 10.0";
}

TEST_F(ParticleFilterTestSuite, TestPredictSlowMove) {
  pf.prediction(0.1, SIGMA_POS, 10.0, 0); // velocity=10m/s or 1m per 0.1sec - should go diagonal

  double x_avg;
  double y_avg;
  getAverageParticleXY(x_avg, y_avg);

  ASSERT_NEAR(x_avg, 10.707, .1) << "X values should be within 10.0 + sqrt(2)/2";
  ASSERT_NEAR(y_avg, 10.707, .1) << "Y values should be within 10.0 + sqrt(2)/2";
}

TEST_F(ParticleFilterTestSuite, TestPredictSlowMoveAndTurn) {
  // velocity=10m/s or 1m per 0.1sec, rotation PI/4 per 0.1 sec. Should be almost straight up.
  pf.prediction(0.1, SIGMA_POS, 20.0, 10*M_PI/4);

  double x_avg;
  double y_avg;
  getAverageParticleXY(x_avg, y_avg);

  ASSERT_NEAR(x_avg, 10.7, .1);
  ASSERT_NEAR(y_avg, 11.7, .1);
}
