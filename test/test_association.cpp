//
// Created by Andrii Garkavyi on 12/18/17.
//

#include "gtest/gtest.h"
#include "particle_filter.h"

using namespace std;

double sigma_landmark[2] = {0.3, 0.3};

class AssociationTestSuite : public :: testing::Test {
protected:
  ParticleFilter pf;

  Particle p;
  vector<LandmarkObs> observations;
  Map map;


  virtual void SetUp() {
    //cout << "Setup call " << endl;
    //pf.init(10.0, 10.0, M_PI/4, SIGMA_POS);

    p.x = 10.0;
    p.y = 10.0;
    p.theta = M_PI/4;

    map.landmark_list.push_back({1, 7.0, 7.0});
    map.landmark_list.push_back({2, 9.0, 10.0});
    map.landmark_list.push_back({3, 15.0, 5.0});
    map.landmark_list.push_back({4, 14.0, 12.0});
//    cout << "1st landmark: " << map.landmark_list[0].id_i << endl;
  }


};

TEST_F(AssociationTestSuite, TestAssoc0) {

  pf.associateObservationsWithLandmarks(p, observations, map.landmark_list, sigma_landmark);

  ASSERT_EQ(p.associations.size(), 0);
}

TEST_F(AssociationTestSuite, TestAssoc) {
  observations.push_back({11, 15.3, 5.3}); //very close

  pf.associateObservationsWithLandmarks(p, observations, map.landmark_list, sigma_landmark);

  ASSERT_EQ(p.associations.size(), 1);
  ASSERT_EQ(p.associations[0], 3); //id
  ASSERT_NEAR(p.weight, 0.65, 1e-2);
}

TEST_F(AssociationTestSuite, TestAssoc2Obs) {
  observations.push_back({11, 15.3, 15.0});
  observations.push_back({12, 13.0, 15.0});
  observations.push_back({13, 8.0, 8.0});

  pf.associateObservationsWithLandmarks(p, observations, map.landmark_list, sigma_landmark);

  ASSERT_EQ(p.associations.size(), 3);
  ASSERT_EQ(p.associations[0], 4); //id
  ASSERT_EQ(p.associations[1], 4); //id
  ASSERT_EQ(p.associations[2], 1); //id
}

//========= Range filtering ==================

TEST_F(AssociationTestSuite, TestFilter1) {
  vector<Map::single_landmark_s> landmarks = map.landmarksInRange(1.0, 10.0, 10.0);

  ASSERT_EQ(landmarks.size(), 1);
  ASSERT_EQ(landmarks[0].id_i, 2);
}

TEST_F(AssociationTestSuite, TestFilter0) {
  vector<Map::single_landmark_s> landmarks = map.landmarksInRange(0.5, 10.0, 10.0);

  ASSERT_EQ(landmarks.size(), 0);
}
