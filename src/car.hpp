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
#include "helper.hpp"

using namespace std;

enum State {
  STAY, CL, CR
};

struct Trajectory {
  vector<Position> pos;
  double cost;
};

class OtherCar {
public:
  OtherCar(int id, double x, double y, double vx, double vy, double s, double d, vector<double> &maps_s, vector<double> &maps_x, vector<double> &maps_y);
  OtherCar() {};
  
//  bool valid = false;
  
  int id;
  double x;
  double y;
  double vx;
  double vy;
  double s;
  double d;
  
  double v_total;
  
  Trajectory traj;
  
  double min_dist(Trajectory prev_traj, Trajectory new_traj);
  
  string toString();
};

class Car {
public:
//  Car() {}
  
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
  
  OtherCar car_in_lane;
  double car_in_lane_dist;
  
  vector<OtherCar> cars_left;
  vector<OtherCar> cars_right;
  
  // Creates a set of candidate trajectories.
  void create_candidate_trajectories(Position start_pos, int no_points, long long start_time);
  
  // Calculates a minimum jerk trajectory for the given inputs.
  Trajectory calculateTrajectory(Position start_pos, double target_d, double desired_v, int no_points);
  Trajectory calculateFallbackTrajectory(Position start_pos, double desired_v, int no_points);
  
  // Calculates the cost for a given trajectory. Returns false, if trajectory is not feasible.
  bool evaluate_trajectory(Trajectory traj);
  
  // Determines the best trajectory according to cost function.
  void select_best_trajectory();
  
  // Return vectors of xy coordinates for the best trajectory.
  vector<double> get_best_trajectory_x();
  vector<double> get_best_trajectory_y();

};

#endif /* car_hpp */
