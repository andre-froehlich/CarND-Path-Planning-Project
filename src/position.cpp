//
//  position.cpp
//  Path_Planning
//
//  Created by Andre Fröhlich on 28.07.17.
//
//

#include <stdio.h>
#include <string>
#include <vector>
#include "position.hpp"
#include "helper.hpp"

using namespace std;

double Position::get_x() {
  return x;
}

double Position::get_y() {
  return y;
}

double Position::get_v_x() {
  return v_x;
}

double Position::get_v_y() {
  return v_y;
}

double Position::get_v_total() {
  return v_total;
}

double Position::get_a_x() {
  return a_x;
}

double Position::get_a_y() {
  return a_y;
}

double Position::get_a_total() {
  return a_total;
}

double Position::get_theta() {
  return theta;
}

double Position::get_v_theta() {
  return v_theta;
}

double Position::get_a_theta() {
  return a_theta;
}

bool Position::is_xy_init() {
  return xy_init;
}

bool Position::is_v_xy_init() {
  return v_xy_init;
}

void Position::set_xy(double x, double y) {
  this->x = x;
  this->y = y;
  xy_init = true;
}

void Position::set_v_total(double v) {
  v_total = v;
}

void Position::set_v_xy(double vx, double vy) {
  v_x = vx;
  v_y = vy;
  v_xy_init = true;
}

void Position::set_a_xy(double ax, double ay) {
  a_x = ax;
  a_y = ay;
  a_xy_init = true;
}

void Position::set_theta(double t) {
  theta = t;
}

void Position::calc_xy(double s, double d, vector<double> &map_waypoints_s, vector<double> &map_waypoints_x, vector<double> &map_waypoints_y) {
  this->s = s;
  this->d = d;
  
  vector<double> coord = getXY(s, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  x = coord[0];
  y = coord[1];
  
  xy_init = true;
}

void Position::calc_v_total() {
  v_total = sqrt(v_x * v_x + v_y * v_y);
}

void Position::calc_a_total() {
  a_total = sqrt(a_x * a_x + a_y * a_y);
}

void Position::calc_v_xy(Position &prev) {
  v_x = (x - prev.x) / dt;
  v_y = (y - prev.y) / dt;
  v_xy_init = true;
}

void Position::calc_a_xy(Position &prev) {
  a_x = (v_x - prev.v_x) / dt;
  a_y = (v_y - prev.v_y) / dt;
}

double get_angle(double x, double y) {
  if (x == 0.0) {
    if (y >= 0.0) {
      return 0.0;
    } else {
      return pi();
    }
  }
  
  if (y == 0.0) {
    if (x >= 0.0) {
      return 0.5 * pi();
    } else {
      return 1.5 * pi();
    }
  }
  
  if (x >= 0.0) {
    if (y >= 0.0) {   // upper right quadrant
      return atan(fabs(x) / fabs(y));
    } else {          // lower right quadrant
      return atan(fabs(y) / fabs(x)) + 0.5 * pi();
    }
  } else {
    if (y >= 0.0) {   // upper left quadrant
      return atan(fabs(y) / fabs(x)) + 1.5 * pi();
    } else {          // lower left qudrant
      return atan(fabs(x) / fabs(y)) + pi();
    }
  }
}

void Position::calc_theta(Position &prev) {
  double dx = x - prev.x;
  double dy = y - prev.y;
  theta = get_angle(dx, dy);
}

void Position::calc_v_theta() {
//  v_theta = atan(v_y / v_x);
  v_theta = get_angle(v_x, v_y);
}

void Position::calc_a_theta() {
//  a_theta = atan(a_y / a_x);
  a_theta = get_angle(a_x, a_y);
}

void Position::calc_sd(vector<double> maps_x, vector<double> maps_y) {
  vector<double> f = getFrenet(x, y, theta, maps_x, maps_y);
  s = f[0];
  d = f[1];
}


void Position::safety_adjust_v() {
  if (v_total > 20.0) {
    v_x = 20.0 * cos(theta);
    v_y = 20.0 * sin(theta);
    v_total = 20.0;
  }
}

string Position::toString() {
  return
    "s=" + to_string(s) + " / d=" + to_string(d) +
    " / x=" + to_string(x) + " / y=" + to_string(y) +
    " / v_x=" + to_string(v_x) + " / v_y=" + to_string(v_y) +
    " / v_total=" + to_string(v_total) +
    " / a_x=" + to_string(a_x) + " / a_y=" + to_string(a_y) +
    " / a_total=" + to_string(a_total) +
//    " / v_theta=" + to_string(v_theta) + " / a_theta=" + to_string(a_theta)
  
  "";
}

