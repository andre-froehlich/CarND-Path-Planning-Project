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

double Position::get_a_x() {
  return a_x;
}

double Position::get_a_y() {
  return a_y;
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

void Position::calc_xy(double s, double d, vector<double> &map_waypoints_s, vector<double> &map_waypoints_x, vector<double> &map_waypoints_y) {  
  vector<double> coord = getXY(s, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  x = coord[0];
  y = coord[1];
  
  xy_init = true;
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

string Position::toString() {
  return "x=" + to_string(x) + " / y=" + to_string(y) + " / v_x=" + to_string(v_x) + " / v_y=" + to_string(v_y);
}

