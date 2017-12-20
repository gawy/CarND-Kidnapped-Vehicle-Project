/*
 * map.h
 *
 *  Created on: Dec 12, 2016
 *      Author: mufferm
 */

#ifndef MAP_H_
#define MAP_H_

class Map {
public:
	
	struct single_landmark_s{

		int id_i ; // Landmark ID
		float x_f; // Landmark x-position in the map (global coordinates)
		float y_f; // Landmark y-position in the map (global coordinates)
	};

	std::vector<single_landmark_s> landmark_list ; // List of landmarks in the map

  /**
   * Filter out landmarks that are outside circle with radius=`range` and cetner at (x,y).
   * @param range radius of circle used for filtering
   * @param x center x of circle used for filtering
   * @param y center y of circle used for filtering
   * @return vector of landmarks that fit within range
   */
  inline std::vector<single_landmark_s> landmarksInRange(double range, double x, double y) const {
		std::vector<single_landmark_s> res;

		for (int i = 0; i < landmark_list.size(); ++i) {
			single_landmark_s l = landmark_list[i];
			double r = sqrt( (x-l.x_f)*(x-l.x_f) + (y-l.y_f)*(y-l.y_f) );
			if (r <= range) {
				res.push_back(l);
			}
		}

		return res;
	}

};



#endif /* MAP_H_ */
