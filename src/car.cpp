//
//  car.cpp
//  Path_Planning
//
//  Created by Andre Fröhlich on 22.07.17.
//
//

#include <iostream>
#include <algorithm>
#include "helper.hpp"
#include "car.hpp"
#include "position.hpp"
#include "spline.h"

using namespace std;

OtherCar::OtherCar(int id, double x, double y, double vx, double vy, double s, double d, vector<double> &maps_s, vector<double> &maps_x, vector<double> &maps_y) {
  this->id = id;
  this->x = x;
  this->y = y;
  this->vx = vx;
  this->vy = vy;
  this->s = s;
  this->d = d;
  
  v_total = sqrt(vx * vx + vy * vy);
//  Position start_pos;
//  start_pos.set_xy(x, y);
//  traj.pos.push_back(start_pos);
//  double v_incr = v_total * dt;
//  
//  for (int i=0; i<desired_path_len; i++) {
//    s += v_incr;
//    vector<double> f = getXY(s, d, maps_s, maps_x, maps_y);
//    Position pos;
//    pos.set_xy(f[0], f[1]);
//    traj.pos.push_back(pos);
//  }
}

double OtherCar::min_dist(Trajectory prev_traj, Trajectory new_traj) {
  double result = numeric_limits<double>::max();
  int i = 0;
  for (Position p : prev_traj.pos) {
    double dist_x = p.get_x() - traj.pos[i].get_x();
    double dist_y = p.get_y() - traj.pos[i].get_y();
    double dist = sqrt(dist_x * dist_x + dist_y * dist_y);
    if (dist < result) result = dist;
    i++;
  }
  
  for (Position p : new_traj.pos) {
    double dist_x = p.get_x() - traj.pos[i].get_x();
    double dist_y = p.get_y() - traj.pos[i].get_y();
    double dist = sqrt(dist_x * dist_x + dist_y * dist_y);
    if (dist < result) result = dist;
    i++;
  }
  
  return result;
}

string OtherCar::toString() {
  return "OtherCar id=" + to_string(id) + " / x=" + to_string(x) + " / y=" + to_string(y) + " / vx=" +
  to_string(vx) + " / vy=" + to_string(vy) + " / s=" + to_string(s) + " / d=" + to_string(d);
}

//int max_trajectory_length = 200;
vector<double> Car::get_best_trajectory_x() {
  vector<double> result;
  
  for (int i=0; i < best_trajectory.prev_path_len; i++) {
    result.push_back(previous_trajectory.pos[i].get_x());
  }
  for (Position p : best_trajectory.pos) {
    result.push_back(p.get_x());
  }
  
  /*
  int i = 0;
  for (Position p : previous_trajectory.pos) {
    i++;
    if (i > max_trajectory_length) break;
    result.push_back(p.get_x());
  }
  for (Position p : best_trajectory.pos) {
    i++;
    if (i > max_trajectory_length) break;
    result.push_back(p.get_x());
  }
   */
  
  return result;
}

vector<double> Car::get_best_trajectory_y() {
  vector<double> result;
  
  for (int i=0; i < best_trajectory.prev_path_len; i++) {
    result.push_back(previous_trajectory.pos[i].get_y());
  }
  for (Position p : best_trajectory.pos) {
    result.push_back(p.get_y());
  }
  
  /*
  int i = 0;
  for (Position p : previous_trajectory.pos) {
    i++;
    if (i > max_trajectory_length) break;
    result.push_back(p.get_y());
  }
  for (Position p : best_trajectory.pos) {
    i++;
    if (i > max_trajectory_length) break;
    result.push_back(p.get_y());
  }
   */
  return result;
}

Trajectory Car::calculateTrajectory(Position start_pos, double end_d, double desired_v, int no_points) {
  const double max_a = 5.0;
  double T = dt * no_points;
  Trajectory result;
  
  // Determine end position and speeds
  double target_total_v = min(desired_v, start_pos.get_v_total() + max_a * T);
  double target_dist = start_pos.get_v_total() * T + 0.5 * (target_total_v - start_pos.get_v_total()) * T;
  vector<double> f = getFrenet(start_pos.get_x(), start_pos.get_y(), start_pos.get_theta(), map_waypoints_x, map_waypoints_y);
  double end_s = f[0] + target_dist;
  if (end_s > 6945.554) end_s -= 6945.554;
  
  Position end_pos, end_pos_1;
  end_pos.calc_xy(end_s, end_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  end_pos_1.calc_xy(end_s - target_total_v * dt, end_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  end_pos.calc_theta(end_pos_1);
  double target_v_x = target_total_v * sin(end_pos.get_theta());
  double target_v_y = target_total_v * cos(end_pos.get_theta());
  
  // Calculate JMT coefficients
//  vector<double> start_x = {start_pos.get_x(), start_pos.get_v_x(), start_pos.get_a_x()};
  vector<double> start_x = {start_pos.get_x(), start_pos.get_v_x(), 0.0};
  vector<double> end_x = {end_pos.get_x(), target_v_x, 0.0};
//  vector<double> start_y = {start_pos.get_y(), start_pos.get_v_y(), start_pos.get_a_y()};
  vector<double> start_y = {start_pos.get_y(), start_pos.get_v_y(), 0.0};
  vector<double> end_y = {end_pos.get_y(), target_v_y, 0.0};
  vector<double> coeffs_x = JMT(start_x, end_x, T);
  vector<double> coeffs_y = JMT(start_y, end_y, T);
  
  /*
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
   */
  
  Position prevPos = start_pos;
  for (double t = dt; t <= T; t+=dt) {
    // Calculate powers of t to reduce computation
    double tt = t * t;
    double ttt = tt * t;
    double tttt = ttt * t;
    double ttttt = tttt * t;
    
    // Calculate trajectory point and add it to trajectory
    double x = coeffs_x[0] + coeffs_x[1] * t + coeffs_x[2] * tt + coeffs_x[3] * ttt + coeffs_x[4] * tttt + coeffs_x[5] * ttttt;
    double y = coeffs_y[0] + coeffs_y[1] * t + coeffs_y[2] * tt + coeffs_y[3] * ttt + coeffs_y[4] * tttt + coeffs_y[5] * ttttt;
    Position newPos;
    newPos.set_xy(x, y);
    newPos.calc_v_xy(prevPos);
    newPos.calc_v_total();
    newPos.calc_a_xy(prevPos);
    newPos.calc_a_total();
    
    // Check, if trajectory is feasible
    if (newPos.get_v_total() > speed_limit_real || newPos.get_a_total() > 8.0) {
      result.cost = numeric_limits<double>::max();
      return result;
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
    if (p.get_v_total() > speed_limit_real) {
      return false;
    }
    
    // Check AccTotal
//    double a_total = sqrt(p.get_a_x() * p.get_a_x() + p.get_a_y() * p.get_a_y());
//    if (a_total > a_max) {
//      return false;
//    }
    
    // Check AccT
    // Check AccN
    // Check Jerk
  }
  
//  for (OtherCar o : other_cars) {
////    double min_dist = o.min_dist(previous_trajectory, traj);
////    cout << "min distance to " << o.id << ": " << min_dist << endl;
//    cout << o.toString() << endl;
//  }
  
  // TODO: Check for collisions
  
  
  // TODO: calculate cost
  
  traj.cost = 1;
  return true;
}

void printTrajectory(Trajectory t) {
  for (Position p : t.pos) {
    cout << p.toString() << endl;
  }
}

int debug_counter = 1;

Trajectory Car::calculateFallbackTrajectory(Position start_pos, double desired_v, double desired_d, int no_points) {
  assert(desired_d >= 1.5 && desired_d <= 10.5);
  
  debug_counter++;
  
  bool debug = (debug_counter == -3);
  if (debug) {
    cout << "start_pos: " << start_pos.toString() << endl;
    cout << "desired_v=" << desired_v << " / desired_d=" << desired_d << endl;
  }
  
  Trajectory result;
  result.prev_path_len = desired_path_len - no_points;
  result.target_lane = getLane(desired_d);
  
  const double max_a = 3.0;
  double T = dt * no_points;
  double target_total_v = min(desired_v, start_pos.get_v_total() + max_a * T);
  
  // Determine k parameters for d change logistic function
//  const double k = (2 / T) * log(desired_d / start_pos.d - 1.0);
//  const double delta_d = desired_d - start_pos.d;
  
  Position prevPos = start_pos;
  for (double t = dt; t <= T; t+=dt) {
    double new_d;
//    double new_d = delta_d / (1 + exp(k * (t - T /2))) + start_pos.d;
    //    double new_d = start_pos.d;
    
    double delta_d = desired_d - prevPos.d;
    if (fabs(delta_d) > 0.1) {
      new_d = prevPos.d + delta_d * 0.02;
    } else {
      new_d = prevPos.d;
    }
    
    double new_v;
    if (prevPos.get_v_total() <= target_total_v) {
      new_v = min(target_total_v, prevPos.get_v_total() + max_a * dt);
    } else {
      new_v = max(target_total_v, prevPos.get_v_total() - max_a * dt);
    }
    
    if (debug) {
      cout << "start_pos: " << prevPos.toString() << endl;
      cout << "new_d=" << new_d << " / new_v=" << new_v << endl;
    }
    
    Position newPos;
    double new_s = prevPos.s + new_v * dt;
    newPos.calc_xy(new_s, new_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    newPos.calc_v_xy(prevPos);
    newPos.calc_v_theta();
    
    double new_v_theta = newPos.get_v_theta();
    bool invalid;
    do {
      invalid = false;
      
      if (debug) {
        cout << "new_v_theta=" << new_v_theta << endl;
      }
      
      double vx = new_v * sin(new_v_theta);
      double vy = new_v * cos(new_v_theta);
      
      double corrected_x = prevPos.get_x() + vx * dt;
      double corrected_y = prevPos.get_y() + vy * dt;
      
      newPos.set_xy(corrected_x, corrected_y);
      newPos.v_total = new_v;
      newPos.calc_theta(prevPos);
      newPos.calc_sd(map_waypoints_x, map_waypoints_y);
      
      newPos.calc_v_xy(prevPos);
      newPos.calc_v_theta();
      newPos.calc_a_xy(prevPos);
      newPos.calc_a_total();
      
      if (fabs(newPos.get_a_total()) > 8.0) {
        invalid = true;
        double d_v_theta = newPos.get_v_theta() - prevPos.get_v_theta();
        new_v_theta = newPos.get_v_theta() - d_v_theta * 0.9;
      }
      
    } while (invalid);
    
    result.pos.push_back(newPos);
    prevPos = newPos;
  }
  
  result.cost = numeric_limits<double>::max() / 2.0;
  return result;
}

void Car::create_candidate_trajectories(Position start_pos, int no_points, Position min_start_pos, int min_no_points, long long start_time) {
  debug_counter++;
//  best_trajectory.pos.clear();
  
  cout << "Current Lane=" << current_lane << " / Target Lane=" << target_lane;
  cout << " / no_points=" << no_points << endl;
  cout << "change_left_blocked=" << change_left_blocked << " / change_right_blocked=" << change_right_blocked << endl;
  
  cout << "Car ahead: " << target_lane_car_ahead.toString() << " / dist=" << target_lane_car_ahead_dist << endl;
  cout << "Car left : " << target_left_lane_car_ahead.toString() << " / dist=" << target_left_lane_car_ahead_dist << endl;
  cout << "Car right: " << target_right_lane_car_ahead.toString() << " / dist=" << target_right_lane_car_ahead_dist << endl;
  
  // TODO: Adjust speed
  double target_d;
  double target_v;
  if (current_lane != target_lane) {  // in between lanes, finish previous trajectory
    target_d = target_lane;
    if (target_lane_car_ahead_dist < 50.0) {
      target_v = target_lane_car_ahead.v_total - 1.0;
    } else {
      target_v = speed_limit;
    }
//    best_trajectory = calculateFallbackTrajectory(start_pos, speed_limit, target_lane, no_points);
  } else {
    if (target_lane_car_ahead_dist > 100.0) {   // no car ahead, just keep lane
      target_d = target_lane;
      target_v = speed_limit;
//      best_trajectory = calculateFallbackTrajectory(start_pos, speed_limit, target_lane, no_points);
    } else {                          // car ahead
      if (!change_right_blocked) {
        target_d = target_lane + 4;
        if (target_right_lane_car_ahead_dist < 50.0) {
          target_v = target_right_lane_car_ahead.v_total - 1.0;
        } else {
          target_v = speed_limit;
        }
//        best_trajectory = calculateFallbackTrajectory(start_pos, speed_limit, target_lane + 4, no_points);
      } else if (!change_left_blocked) {
        target_d = target_lane - 4;
        if (target_left_lane_car_ahead_dist < 50.0) {
          target_v = target_left_lane_car_ahead.v_total - 1.0;
        } else {
          target_v = speed_limit;
        }
//        best_trajectory = calculateFallbackTrajectory(start_pos, speed_limit, target_lane - 4, no_points);
      } else {
        target_d = target_lane;
        if (target_lane_car_ahead_dist < 50.0) {
          target_v = target_lane_car_ahead.v_total - 1.0;
        } else {
          target_v = speed_limit;
        }
//        best_trajectory = calculateFallbackTrajectory(start_pos, speed_limit, target_lane, no_points);
      }
    }
  }
  
  best_trajectory = calculateFallbackTrajectory(start_pos, target_v, target_d, no_points);
  
  if (best_trajectory.pos.size() == 0) exit(13);
  
  
  /*
  candidate_trajectories.clear();
  
  // Calculate trajectory without lane change
  double desired_speed;
  
  if (car_in_lane_dist < 100.0) {
    desired_speed = car_in_lane.v_total;
  } else if (car_in_lane_dist < 50.0) {
    desired_speed = car_in_lane.v_total - 1.0;
  } else {
    desired_speed = speed_limit;
  }
  //candidate_trajectories.push_back(calculateFallbackTrajectory(start_pos, desired_speed, no_points));

  Trajectory traj;
  
  if (debug_counter == 6) {
    traj = calculateFallbackTrajectory(min_start_pos, desired_speed, Lane::RIGHT, min_no_points);
  } else if (debug_counter >= 35 && debug_counter<50) {
    traj = calculateFallbackTrajectory(min_start_pos, desired_speed, Lane::LEFT, min_no_points);
  } else {
    traj = calculateFallbackTrajectory(start_pos, desired_speed, current_lane, no_points);
  }
  
//  Lane desired_lane = current_lane;
//  if (car_in_lane_dist < 200.0) {
//    if (current_lane == MIDDLE) {
//      desired_lane = LEFT;
//    } else {
//      desired_lane = MIDDLE;
//    }
//    traj = calculateFallbackTrajectory(min_start_pos, desired_speed, desired_lane, min_no_points);
//  } else {
//    traj = calculateFallbackTrajectory(min_start_pos, desired_speed, desired_lane, min_no_points);
//  }

  best_trajectory = traj;
   */
  
  // Set target lane
  target_lane = best_trajectory.target_lane;
  
  cout << endl;
}

