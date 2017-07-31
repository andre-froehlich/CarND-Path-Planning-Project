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

vector<double> Car::get_trajectory_x() {
  vector<double> result;
  for (Position p : traj.pos) {
    result.push_back(p.get_x());
  }
  return result;
}

vector<double> Car::get_trajectory_y() {
  vector<double> result;
  for (Position p : traj.pos) {
    result.push_back(p.get_y());
  }
  return result;
}

void Car::calculateTrajectory(Position start_pos, double start_theta, double current_speed) {
  const int no_points = 50;
  const double a_max = 5.0;
  double T = dt * no_points;
  
//  double target_total_v = speed_limit;
//  double target_dist = 20.0;
  
  double target_total_v = min(speed_limit, current_speed + a_max * T);
  double target_dist = current_speed * T + 0.5 * (target_total_v - current_speed) * T;

  cout << "Current v=" << current_speed << " / Target v=" << target_total_v << " / dist=" << target_dist << endl;
  
  vector<double> f = getFrenet(start_pos.get_x(), start_pos.get_y(), start_theta, map_waypoints_x, map_waypoints_y);
  double end_s = f[0] + target_dist;
  if (end_s > 6945.554) end_s -= 6945.554;
  double end_d = 6.0; //f[1];
  
  Position end_pos;
  end_pos.calc_xy(end_s, end_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  
  Position end_pos_1;
  end_pos_1.calc_xy(end_s - 0.4, end_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
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
    
    if (prevPos.is_xy_init()) {
      newPos.calc_v_xy(prevPos);
      if (prevPos.is_v_xy_init()) {
        newPos.calc_a_xy(prevPos);
      }
    }
    
    traj.pos.push_back(newPos);
    prevPos = newPos;
  }
}
