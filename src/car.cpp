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
//#include "othercar.hpp"

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


int max_trajectory_length = 200;
vector<double> Car::get_best_trajectory_x() {
  vector<double> result;
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
  return result;
}

vector<double> Car::get_best_trajectory_y() {
  vector<double> result;
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
  return result;
}

Trajectory Car::calculateTrajectory(Position start_pos, double target_d, double desired_v, int no_points) {
//  const int no_points = 50;
  const double max_a = 5.0;
  double T = dt * no_points;
  
  Trajectory result;
  
  start_pos.safety_adjust_v();
  
  double target_total_v = min(desired_v, start_pos.get_v_total() + max_a * T);
  double target_dist = start_pos.get_v_total() * T + 0.5 * (target_total_v - start_pos.get_v_total()) * T;

//  cout << "Current v=" << start_pos.get_v_total() << " / Target v=" << target_total_v << " / dist=" << target_dist << endl;
  
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

  
//  vector<double> start_x = {start_pos.get_x(), start_pos.get_v_x(), start_pos.get_a_x()};
  vector<double> start_x = {start_pos.get_x(), start_pos.get_v_x(), 0.0};
  vector<double> end_x = {end_pos.get_x(), target_v_x, 0.0};
//  vector<double> start_y = {start_pos.get_y(), start_pos.get_v_y(), start_pos.get_a_y()};
  vector<double> start_y = {start_pos.get_y(), start_pos.get_v_y(), 0.0};
  vector<double> end_y = {end_pos.get_y(), target_v_y, 0.0};
  
//  cout << "Start Pos: " << start_pos.toString() << endl;
//  cout << "End Pos  : " << end_pos.toString() << endl;
  
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
//    cout << "LOOP START: t=" << t << " / T=" << T << endl;
    
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
        newPos.calc_a_total();
      }
    }
    
    if (newPos.get_v_total() <= speed_limit) {
      result.pos.push_back(newPos);
      prevPos = newPos;
    } else {
//      cout << "INV START: t=" << t << " / T=" << T << endl;
      t -= dt;
      t -= 0.002;
      T -= 0.002;
//      cout << "INV END: t=" << t << " / T=" << T << endl;
    }
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

void Car::create_candidate_trajectories(Position start_pos, int no_points) {
//  cout << start_pos.toString() << endl;
  
  candidate_trajectories.clear();
  
  // Calculate trajectory without lane change
  Trajectory traj;
  if (car_in_lane.valid) {
    traj = calculateTrajectory(start_pos, current_lane, car_in_lane.v_total - 1.0, no_points);
  } else {
    traj = calculateTrajectory(start_pos, current_lane, speed_limit, no_points);
  }
  
  best_trajectory = traj;
  
  // TODO: Calculate some trajectories...
//  for (int dv=0; dv<=0; dv+=2) {
//    Trajectory traj = calculateTrajectory(start_pos, 6.0, speed_limit - dv, no_points);
////    cout << "Traj len=" << traj.pos.size() << endl;
//    if (evaluate_trajectory(traj)) {
//      candidate_trajectories.push_back(traj);
//    } else {
//      printTrajectory(traj);
//      exit(0);
//    }
//  }
  
//  if (candidate_trajectories.size() == 0) {
//    exit(0);
//  }
  
  // Select best trajectory
//  if (candidate_trajectories.size() > 0) {
//    best_trajectory = candidate_trajectories[0];
//  } else {
//    best_trajectory.pos.clear();
//  }
//  cout << best_trajectory.pos[best_trajectory.pos.size() - 1]
}
