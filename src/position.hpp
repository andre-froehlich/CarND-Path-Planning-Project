//
//  position.hpp
//  Path_Planning
//
//  Created by Andre Fröhlich on 28.07.17.
//
//

#ifndef position_hpp
#define position_hpp

#include <stdio.h>
#include <string>

using namespace std;

class Position {
private:
  double x;
  double y;
  
  double v_total;
  double v_x;
  double v_y;
  double a_x;
  double a_y;
  
  double theta;
  
  bool xy_init = false;
  bool v_xy_init = false;
  bool a_xy_init = false;
  
public:
  double get_v_total();
  double get_x();
  double get_y();
  double get_v_x();
  double get_v_y();
  double get_a_x();
  double get_a_y();
  double get_theta();
  
  bool is_xy_init();
  bool is_v_xy_init();
  bool is_a_xy_init();
  
  void set_xy(double x, double y);
  void set_v_xy(double vx, double vy);
  void set_a_xy(double ax, double ay);
  void set_v_total(double v);
  void set_theta(double t);
  
  void calc_xy(double s, double d, vector<double> &map_waypoints_s, vector<double> &map_waypoints_x, vector<double> &map_waypoints_y);
  void calc_v_total();
  void calc_v_xy(Position &prev);
  void calc_a_xy(Position &prev);
  void calc_theta(Position &prev);
  
  string toString();
};

#endif /* position_hpp */
