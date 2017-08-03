//
//  car.hpp
//  Path_Planning
//
//  Created by Andre Fröhlich on 22.07.17.
//
//

#ifndef car_hpp
#define car_hpp

#include <stdio.h>
#include <vector>
#include "position.hpp"

using namespace std;



enum Lane {
  LEFT = 2,
  MIDDLE = 6,
  RIGHT = 10
};

enum State {
  STAY, CL, CR
};

struct Trajectory {
  vector<Position> pos;
  double cost;
};

class Car {
public:
  Lane current_lane = MIDDLE;
  State current_state = STAY;
  
  Trajectory previous_trajectory;
  Trajectory best_trajectory;
  vector<Trajectory> candidate_trajectories;
  
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
  
  // Creates a set of candidate trajectories.
  void create_candidate_trajectories(Position start_pos, int no_points);
  
  // Calculates a minimum jerk trajectory for the given inputs.
  Trajectory calculateTrajectory(Position start_pos, double target_d, double desired_v, int no_points);
  
  // Calculates the cost for a given trajectory. Returns false, if trajectory is not feasible.
  bool evaluate_trajectory(Trajectory traj);
  
  // Determines the best trajectory according to cost function.
  void select_best_trajectory();
  
  // Return vectors of xy coordinates for the best trajectory.
  vector<double> get_best_trajectory_x();
  vector<double> get_best_trajectory_y();

};

#endif /* car_hpp */
