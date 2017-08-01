//
//  car.cpp
//  Path_Planning
//
//  Created by Andre Fröhlich on 22.07.17.
//
//

#include <iostream>
#include "helper.hpp"
#include "car.hpp"
#include "position.hpp"

using namespace std;

vector<double> Car::get_best_trajectory_x() {
  vector<double> result;
  for (Position p : previous_trajectory.pos) {
    result.push_back(p.get_x());
  }
  for (Position p : best_trajectory.pos) {
    result.push_back(p.get_x());
  }
  return result;
}

vector<double> Car::get_best_trajectory_y() {
  vector<double> result;
  for (Position p : previous_trajectory.pos) {
    result.push_back(p.get_y());
  }
  for (Position p : best_trajectory.pos) {
    result.push_back(p.get_y());
  }
  return result;
}

Trajectory Car::calculateTrajectory(Position start_pos, double target_d, double desired_v) {
  const int no_points = 50;
  const double max_a = 5.0;
  double T = dt * no_points;
  
  Trajectory result;
  
  double target_total_v = min(desired_v, start_pos.get_v_total() + max_a * T);
  double target_dist = start_pos.get_v_total() * T + 0.5 * (target_total_v - start_pos.get_v_total()) * T;

  cout << "Current v=" << start_pos.get_v_total() << " / Target v=" << target_total_v << " / dist=" << target_dist << endl;
  
  vector<double> f = getFrenet(start_pos.get_x(), start_pos.get_y(), start_pos.get_theta(), map_waypoints_x, map_waypoints_y);
  double end_s = f[0] + target_dist;
  if (end_s > 6945.554) end_s -= 6945.554;
  double end_d = 6.0; //f[1]; // target_d
  
  Position end_pos;
  end_pos.calc_xy(end_s, end_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  
  Position end_pos_1;
  end_pos_1.calc_xy(end_s - target_total_v * dt, end_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  double end_theta = atan((end_pos.get_y() - end_pos_1.get_y()) / (end_pos.get_x() - end_pos_1.get_x()));
  
  double target_v_x = target_total_v * cos(end_theta);
  double target_v_y = target_total_v * sin(end_theta);

  
  vector<double> start_x = {start_pos.get_x(), start_pos.get_v_x(), start_pos.get_a_x()};
  vector<double> end_x = {end_pos.get_x(), target_v_x, 0.0};
  vector<double> start_y = {start_pos.get_y(), start_pos.get_v_y(), start_pos.get_a_y()};
  vector<double> end_y = {end_pos.get_y(), target_v_y, 0.0};
  
  cout << "Start Pos: " << start_pos.toString() << endl;
  cout << "End Pos  : " << end_pos.toString() << endl;
  
  vector<double> coeffs_x = JMT(start_x, end_x, T);
  vector<double> coeffs_y = JMT(start_y, end_y, T);
  
//  cout << "X coeffs:";
//  for (int i=0; i<coeffs_x.size(); i++) {
//    cout << "a" << i << "=" << coeffs_x[i] << " / ";
//  }
//  cout << endl;
//  cout << "Y coeffs:";
//  for (int i=0; i<coeffs_y.size(); i++) {
//    cout << "a" << i << "=" << coeffs_y[i] << " / ";
//  }
//  cout << endl;
  
  Position prevPos = start_pos;
  for (double t=dt; t<=T; t+=dt) {
    // Calculate powers of t to reduce computation
    double tt = t * t;
    double ttt = tt * t;
    double tttt = ttt * t;
    double ttttt = tttt * t;
    
    // Calculate trajectory point and add it to trajectory
    Position newPos;
    newPos.set_xy(coeffs_x[0] + coeffs_x[1] * t + coeffs_x[2] * tt + coeffs_x[3] * ttt + coeffs_x[4] * tttt + coeffs_x[5] * ttttt,
                  coeffs_y[0] + coeffs_y[1] * t + coeffs_y[2] * tt + coeffs_y[3] * ttt + coeffs_y[4] * tttt + coeffs_y[5] * ttttt);
    
    // Calculate speed and acceleration for the trajectory point
    if (prevPos.is_xy_init()) {
      newPos.calc_v_xy(prevPos);
      newPos.calc_v_total();
      newPos.calc_theta(prevPos);
      if (prevPos.is_v_xy_init()) {
        newPos.calc_a_xy(prevPos);
      }
    }
    
    result.pos.push_back(newPos);
    prevPos = newPos;
  }
  
  return result;
}

bool Car::evaluate_trajectory(Trajectory traj) {
  // TODO: feasibility checks
  for (Position p : traj.pos) {
    // Check speed
    if (p.get_v_total() >= speed_limit_real) {
      return false;
    }
    
    // Check AccTotal
    double a_total = sqrt(p.get_a_x() * p.get_a_x() + p.get_a_y() * p.get_a_y());
    if (a_total > a_max) {
      return false;
    }
    
    // Check AccT
    // Check AccN
    // Check Jerk
  }
  
  // TODO: Check for collisions
  
  
  // TODO: calculate cost
  
  traj.cost = 1;
  return true;
}

void Car::create_candidate_trajectories(Position start_pos) {
  candidate_trajectories.clear();
  
  // TODO: Calculate some trajectories...
  Trajectory traj = calculateTrajectory(start_pos, 6.0, speed_limit);
  if (evaluate_trajectory(traj)) candidate_trajectories.push_back(traj);
  
  traj = calculateTrajectory(start_pos, 6.0, speed_limit - 5.0);
  if (evaluate_trajectory(traj)) candidate_trajectories.push_back(traj);
  
  traj = calculateTrajectory(start_pos, 6.0, speed_limit - 10.0);
  if (evaluate_trajectory(traj)) candidate_trajectories.push_back(traj);
  
  traj = calculateTrajectory(start_pos, 6.0, speed_limit - 15.0);
  if (evaluate_trajectory(traj)) candidate_trajectories.push_back(traj);
  
  // Select best trajectory
  best_trajectory = candidate_trajectories[0];
}
