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


int max_trajectory_length = 200;
vector<double> Car::get_best_trajectory_x() {
//  cout << "Prev Traj len=" << previous_trajectory.pos.size() << endl;
//  cout << "Best Traj len=" << best_trajectory.pos.size() << endl;
  
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
  
//  cout << "Result len=" << result.size() << endl;
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

bool sorter(vector<double> v1, vector<double> v2) {
  return v1[0] < v2[0];
}

Trajectory Car::calculateTrajectory(Position start_pos, double target_d, double desired_v, int no_points) {
  const double max_a = 5.0;
  double T = dt * no_points;
  
  Trajectory result;
  
//  start_pos.safety_adjust_v();
  
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
  
  cout << "start_pos.x=" << start_pos.get_x() << " / .v_x=" << start_pos.get_v_x() << endl;
  cout << "end_pos.x=" << end_pos.get_x() << " / target_v_x=" << target_v_x << endl;
  cout << "start_pos.y=" << start_pos.get_y() << " / .v_y=" << start_pos.get_v_y() << endl;
  cout << "end_pos.y=" << end_pos.get_y() << " / target_v_y=" << target_v_y << endl;

  
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
  
  double dt_jmt = T / (no_points * 0.1);
  cout << "Number of points=" << no_points << " / dt_jmt=" << dt_jmt << endl;
  double t = 0.0;
  
  int points_added = 0;
  
  vector<double> x_for_spline;
  vector<double> y_for_spline;
  vector<double> t_for_spline;
  
  for (Position p : previous_trajectory.pos) {
    x_for_spline.push_back(p.get_x());
    y_for_spline.push_back(p.get_y());
    t_for_spline.push_back(t);
    
    cout << "x=" << p.get_x() << " / y=" << p.get_y() << " / t=" << t << endl;
    
    t += dt;
  }
  const double offset_t = t - dt;
  
  cout << "*** new traj / offset" << offset_t << endl;
  
  double prev_x = start_pos.get_x();
  double prev_y = start_pos.get_y();
  
  t = dt_jmt;
  while (t <= T) {
    // Calculate powers of t to reduce computation
    double tt = t * t;
    double ttt = tt * t;
    double tttt = ttt * t;
    double ttttt = tttt * t;
    
    // Calculate trajectory point and add it to trajectory
    double x = coeffs_x[0] + coeffs_x[1] * t + coeffs_x[2] * tt + coeffs_x[3] * ttt + coeffs_x[4] * tttt + coeffs_x[5] * ttttt;
    double y = coeffs_y[0] + coeffs_y[1] * t + coeffs_y[2] * tt + coeffs_y[3] * ttt + coeffs_y[4] * tttt + coeffs_y[5] * ttttt;
    double vx = (x - prev_x) / dt_jmt;
    double vy = (y - prev_y) / dt_jmt;
    double v_total = sqrt(vx * vx + vy * vy);
    
    cout << "x=" << x << " / y=" << y << " / t=" << (t + offset_t) << " / v_total=" << v_total;
    
    if (v_total >= 0 && v_total <= speed_limit) {
      x_for_spline.push_back(x);
      y_for_spline.push_back(y);
      t_for_spline.push_back(t + offset_t);
      
      cout << " ADDED";
      
      
      points_added++;
    }
    
    cout << endl;
    
    prev_x = x;
    prev_y = y;
    t += dt_jmt;
  }
  
  tk::spline spline_x, spline_y;
  spline_x.set_points(t_for_spline, x_for_spline, true);
  spline_y.set_points(t_for_spline, y_for_spline, true);
  
  cout << endl << "*** Complete traj" << endl;
  
  for (t = 0.0; t <= T; t+=dt) {
    double x = spline_x(t);
    double y = spline_y(t);
    
    cout << "t=" << t << " / x=" << x << " / y=" << y << endl;
    
    Position p;
    p.set_xy(x, y);
    result.pos.push_back(p);
  }
  
  cout << "Finished" << endl;
  
//  exit(2);

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

Trajectory Car::calculateFallbackTrajectory(Position start_pos, double desired_v, int no_points) {
  Trajectory result;
  
  const double max_a = 3.0;
  double T = dt * no_points;
  double target_total_v = min(desired_v, start_pos.get_v_total() + max_a * T);
  
  cout << "target_tota_v=" << target_total_v << " / desired_v=" << desired_v << " / no_points=" << no_points << endl;
  cout << "Start_pos: " << start_pos.toString() << endl;
  
  Position prevPos = start_pos;
  for (int i=0; i<no_points; i++) {
    
    double new_v;
    if (prevPos.get_v_total() <= target_total_v) {
      new_v = min(target_total_v, prevPos.get_v_total() + max_a * dt);
    } else {
      new_v = max(target_total_v, prevPos.get_v_total() - max_a * dt);
    }
    
    Position newPos;
    double new_s = prevPos.s + new_v * dt;
    newPos.calc_xy(new_s, prevPos.d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    newPos.calc_v_xy(prevPos);
    newPos.calc_v_theta();
    
//    newPos.calc_v_total();
//    newPos.calc_a_xy(prevPos);
//    newPos.calc_a_total();
    
    
//    Position corrected_pos;
    
    /*
    // Speed check
    bool invalid = (newPos.get_v_total() > target_total_v);
    
    // Acceleration check
    if (!invalid) {
      newPos.calc_a_xy(prevPos);
      newPos.calc_a_total();
      invalid = (newPos.get_a_total() > 5.0);
    }
    
    invalid = true;
     */
    
    double new_v_theta = newPos.get_v_theta();
    bool invalid;
    do {
      invalid = false;
      
      /*
//      double m = (newPos.get_v_y() - prevPos.get_v_y()) / (newPos.get_v_x() - prevPos.get_v_x());
//      double vy = new_v * m / sqrt(1 + m * m);
//      double vx = vy / m;
       */
      
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
        
        cout << "*NO PUSH:: " << newPos.toString() << endl;
//        exit(6);
      }
      
    } while (invalid);
    
    cout << "    PUSH:: " << newPos.toString() << endl;
    
    /*
    if (invalid) {
      double m = (newPos.get_v_y() - prevPos.get_v_y()) / (newPos.get_v_x() - prevPos.get_v_x());
      double vy = new_v * m / sqrt(1 + m * m);
      double vx = vy / m;
      
      double corrected_x = prevPos.get_x() + vx * dt;
      double corrected_y = prevPos.get_y() + vy * dt;
      
      // Check?
      corrected_pos.set_xy(corrected_x, corrected_y);
      corrected_pos.v_total = new_v;
      corrected_pos.calc_theta(prevPos);
      corrected_pos.calc_sd(map_waypoints_x, map_waypoints_y);
      
      // Check a_total
      corrected_pos.calc_v_xy(prevPos);
      corrected_pos.calc_a_xy(prevPos);
      corrected_pos.calc_a_total();
      
      cout << "    PUSH:: " << corrected_pos.toString() << endl;
      
      
      
      if (corrected_pos.get_a_total() > 8.0) {
        cout << "*** a_total check failed = " << corrected_pos.get_a_total() << endl;
     
        exit(7);
      }
      
    } else {
      corrected_pos = newPos;
    }
     */
    /*
    double last_x = prevPos.get_x();
    double last_y = prevPos.get_y();
    bool is_pos_invalid;
    double lower_v = new_v - 0.5;
    double upper_v = new_v + 0.5;
    bool is_continuous = true;
    
    
    do {
      is_pos_invalid = false;
      double new_s = prevPos.s + new_v * dt;
      if (is_continuous) {
        newPos.calc_xy(new_s, prevPos.d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
      } else {
        newPos.set_xy((newPos.get_x() + last_x) / 2.0, (newPos.get_y() + last_y) / 2.0);
      }
      
      newPos.calc_v_xy(prevPos);
      newPos.calc_v_total();
      
      double a_total = (newPos.get_v_total() - prevPos.get_v_total()) / dt;
      
      cout << "CHECKING:: " << newPos.toString() << " / a_total=" << a_total << endl;
      cout << "lower_v=" << lower_v << " / new_v=" << new_v << " upper_v=" << upper_v << endl;
      
      // Acceleration check
      bool is_braking = (a_total < 0.0);
      if (fabs(a_total) > 8.0) {
        cout << "*** ACC Test failed and is ";
        if (is_braking) {
          cout << "braking" << endl;
          lower_v = new_v;
        } else {
          cout << "accelerating" << endl;
          upper_v = new_v;
        }
        is_pos_invalid = true;
      }
      
      // Speed check
      if (!is_pos_invalid && !is_braking && (newPos.get_v_total() > target_total_v)) {
        cout << "*** Speed test failed" << endl;
        upper_v = new_v;
        is_pos_invalid = true;
      }
      
      if (!is_continuous) {
        break;
      }
      
      new_v = (lower_v + upper_v) / 2.0;
      
      if (fabs(lower_v - upper_v) < 0.001) {
        is_continuous = false;
        cout << "DISCONTINOUS" << endl;
        break;
      } else {
        last_x = newPos.get_x();
        last_y = newPos.get_y();
      }
      
    } while (is_pos_invalid);
    
    if (is_pos_invalid) {
      cout << "Still invalid!!!" << endl;
      double nx = prevPos.get_x() + prevPos.get_v_x() * dt;
      double ny = prevPos.get_y() + prevPos.get_v_y() * dt;
      newPos.set_xy(nx, ny);
      newPos.calc_v_xy(prevPos);
      newPos.calc_v_total();
      newPos.calc_sd(map_waypoints_x, map_waypoints_y);
//      exit(6);
    }*/
    
    
    result.pos.push_back(newPos);
    prevPos = newPos;
  }
  
  return result;
}

void Car::create_candidate_trajectories(Position start_pos, int no_points) {
//  cout << start_pos.toString() << endl;
  
  candidate_trajectories.clear();
  
  // Calculate trajectory without lane change
  Trajectory traj;
  double desired_speed;
  if (car_in_lane_dist < 5.0) {
    desired_speed = 0.0;
  } else if (car_in_lane_dist < 50.0) {
    desired_speed = 0.46666666667 * car_in_lane_dist - 2.3333333333;
  } else {
    desired_speed = speed_limit;
  }
//  traj = calculateTrajectory(start_pos, current_lane, desired_speed, no_points);
  traj = calculateFallbackTrajectory(start_pos, desired_speed, no_points);
  

  
//  if (car_in_lane_dist < 25.0) {
//    traj = calculateTrajectory(start_pos, current_lane, car_in_lane.v_total - 4.0, no_points);
//  } else if (car_in_lane_dist < 50.0) {
//    traj = calculateTrajectory(start_pos, current_lane, car_in_lane.v_total - 1.0, no_points);
//  } else {
//    traj = calculateTrajectory(start_pos, current_lane, speed_limit, no_points);
//  }
  
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
