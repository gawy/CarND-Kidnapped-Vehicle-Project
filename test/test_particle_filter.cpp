//
// Created by Andrii Garkavyi on 12/8/17.
//

#include "gtest/gtest.h"
#include "particle_filter.h"
#include <math.h>

TEST(ParticleFilter, TestInit) {
  ParticleFilter pf;
  double sigma_pos[3] = {0.3, 0.3, 0.01};
  pf.init(14.25, 0.525, M_PI/2, sigma_pos);
  const int num = 100;

  ASSERT_EQ(pf.particles.size(), num);

//  for (int i=0; i < num; i++) {
//    printParticle(pf.particles[i]);
//  }

}
