//
//  helper.hpp
//  Path_Planning
//
//  Created by Andre Fröhlich on 22.07.17.
//
//

#ifndef helper_hpp
#define helper_hpp

#include <stdio.h>
#include <vector>
#include <math.h>

using namespace std;

const double max_s = 6945.554;
const double speed_limit_real = 22.3; // in m/s, it's a bit less than 50mph
const double speed_limit = 21.0;      // for trajectory calculation purposes
const double a_max = 8.0;
const double dt = 0.02;
const double conv_mph2ms = 0.44704;
const int desired_path_len = 200;
const double min_dist_other = 10.0;
const double safe_dist = 50.0;

constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);

double distance(double x1, double y1, double x2, double y2);
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y);
int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y);

vector<double> JMT(vector< double> start, vector <double> end, double T);

enum Lane {
  NONE = 0,
  LEFT = 2,
  MIDDLE = 6,
  RIGHT = 10
};

Lane getLane(double d);

double get_dist_s(double my_s, double other_s);

#endif /* helper_hpp */
