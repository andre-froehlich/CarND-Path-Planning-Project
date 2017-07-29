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
  
  Trajectory traj;
  
  vector<double> get_trajectory_x();
  vector<double> get_trajectory_y();
  
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
  
  void calculateTrajectory(Position start, double start_theta, double current_speed);

};

#endif /* car_hpp */
