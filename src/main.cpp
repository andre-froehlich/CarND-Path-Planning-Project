#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;
//using namespace std::chrono;

// for convenience
using json = nlohmann::json;

// Constants
const double MPH2MS = 0.44704;
const double DT = 0.02;
const double MAX_S = 6945.554;
const double HALF_MAX_S = MAX_S / 2.0;
const double SPEED_LIMIT = 22.1; //22.3
const double MAX_DV = 0.10;
const double SAVE_DIST_AHEAD = 25.0;
const double SAVE_DIST_BEHIND = -25.0;

// State
int current_lane = 1;
int target_lane = 1;

struct OtherCar {
  bool initialized = false;
  double s, vel, ds;
  vector<bool> occupancies;
};

// convert degrees to radians
double deg2rad(double x) { return x * M_PI / 180; }

int getLane(double d) {
  if (d <= 4.0) return 0;
  if (d <= 8.0) return 1;
  return 2;
}

vector<bool> getLaneOccupancies(double d) {
  bool occupies_left = (d <= 5.5);
  bool occupies_middle = ((d >= 2.5) && (d <= 9.5));
  bool occupies_right = (d >= 6.5);
  return {occupies_left, occupies_middle, occupies_right};
}

// convert frenet to xy coordinates
tk::spline spline_wp_x;
tk::spline spline_wp_y;
tk::spline spline_wp_dx;
tk::spline spline_wp_dy;
vector<double> getXY(double s, double d) {
  double x = spline_wp_x(s);
  x += d * spline_wp_dx(s);
  double y = spline_wp_y(s);
  y += d * spline_wp_dy(s);
  return {x, y};
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int cycle_counter = 0;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../../data/highway_map.csv";

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
    
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  spline_wp_x.set_points(map_waypoints_s, map_waypoints_x);
  spline_wp_y.set_points(map_waypoints_s, map_waypoints_y);
  spline_wp_dx.set_points(map_waypoints_s, map_waypoints_dx);
  spline_wp_dy.set_points(map_waypoints_s, map_waypoints_dy);

  
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
        
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          car_yaw = deg2rad(car_yaw);
          double car_speed = j[1]["speed"];
          car_speed *= MPH2MS;

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
        
          json msgJson;
          
          cycle_counter++;
          
          vector<double> next_x_vals, next_y_vals;
          
          current_lane = getLane(car_d);
          bool in_between_lanes = (car_d < 1.5) || (car_d > 2.5 && car_d < 5.5) || (car_d > 6.5 && car_d < 9.5) || (car_d > 10.5);
          
          vector<bool> my_occupancies = getLaneOccupancies(car_d);
          
          if (cycle_counter < 2) {
            
          } else {
            // *** Unpack sensor fusion ***
            OtherCar car_ahead, car_left_ahead, car_left_behind, car_right_ahead, car_right_behind;
            vector<bool> lane_possible;
            lane_possible.push_back(current_lane != 2);
            lane_possible.push_back(true);
            lane_possible.push_back(current_lane != 0);
            vector<double> lane_speed;
            lane_speed.push_back(SPEED_LIMIT);
            lane_speed.push_back(SPEED_LIMIT);
            lane_speed.push_back(SPEED_LIMIT);
          
            for (int i=0; i<sensor_fusion.size(); i++) {
              auto c = sensor_fusion[i];
              OtherCar car;
              car.initialized = true;
              
              double d = c[6];
              if (d < -2.0 || d > 14.0) continue;
              car.occupancies = getLaneOccupancies(d);

              car.s = c[5];
              double vx = c[3];
              double vy = c[4];
              car.vel = sqrt(vx * vx + vy * vy);
              
              car.ds = car.s - car_s;
              if (car.ds < -HALF_MAX_S) {
                car.ds += MAX_S;
              } else if (car.ds > HALF_MAX_S) {
                car.ds -= MAX_S;
              }
              
              // Lane 0
              if (car.occupancies[0]) {
                if (car.ds >= 0.0 && car.ds <= 50.0) lane_speed[0] = min(lane_speed[0], car.vel);
                if (my_occupancies[0] && (car.ds > 0)) {
                  // Car is ahead
                  if (car_ahead.initialized) {
                    if (car_ahead.ds > car.ds) {
                      car_ahead = car;
                    }
                  } else {
                    car_ahead = car;
                  }
                }
                if (my_occupancies[1] && car.ds >= SAVE_DIST_BEHIND && car.ds <= SAVE_DIST_AHEAD) {
                  // Car is on the left
                  lane_possible[0] = false;
                }
              }
              
              // Lane 1
              if (car.occupancies[1]) {
                if (car.ds >= 0.0 && car.ds <= 50.0) lane_speed[1] = min(lane_speed[1], car.vel);
                if (my_occupancies[1] && (car.ds > 0)) {
                  // Car is ahead
                  if (car_ahead.initialized) {
                    if (car_ahead.ds > car.ds) {
                      car_ahead = car;
                    }
                  } else {
                    car_ahead = car;
                  }
                }
                if (my_occupancies[2] && car.ds >= SAVE_DIST_BEHIND && car.ds <= SAVE_DIST_AHEAD) {
                  // Car is on the left
                  lane_possible[1] = false;
                }
                if (my_occupancies[0] && car.ds >= SAVE_DIST_BEHIND && car.ds <= SAVE_DIST_AHEAD) {
                  // Car is on the right
                  lane_possible[1] = false;
                }
              }
              
              // Lane 2
              if (car.occupancies[2]) {
                if (car.ds >= 0.0 && car.ds <= 50.0) lane_speed[2] = min(lane_speed[2], car.vel);
                if (my_occupancies[2] && (car.ds > 0)) {
                  // Car is ahead
                  if (car_ahead.initialized) {
                    if (car_ahead.ds > car.ds) {
                      car_ahead = car;
                    }
                  } else {
                    car_ahead = car;
                  }
                }
                if (my_occupancies[1] && car.ds >= SAVE_DIST_BEHIND && car.ds <= SAVE_DIST_AHEAD) {
                  // Car is on the right
                  lane_possible[2] = false;
                }
              }
            }
            
            // *** Decide strategy ***
            if (!in_between_lanes) {
              if (current_lane == 0) {
                if (lane_speed[0] >= lane_speed[1]) {
                  target_lane = 0;
                } else {
                  if (lane_possible[1]) {
                    target_lane = 1;
                  } else {
                    target_lane = 0;
                  }
                }
              }
              if (current_lane == 1) {
                if (lane_speed[1] >= lane_speed[0] && lane_speed[1] >= lane_speed[2]) {
                  target_lane = 1;
                } else if (lane_speed[2] >= lane_speed[0] && lane_speed[2] >= lane_speed[1]) {
                  if (lane_possible[2]) {
                    target_lane = 2;
                  } else {
                    target_lane = 1;
                  }
                } else {
                  if (lane_possible[0]) {
                    target_lane = 0;
                  } else {
                    target_lane = 1;
                  }
                }
              }
              if (current_lane == 2) {
                if (lane_speed[2] >= lane_speed[1]) {
                  target_lane = 2;
                } else {
                  if (lane_possible[1]) {
                    target_lane = 1;
                  } else {
                    target_lane = 2;
                  }
                }
              }
            }
            
            double target_vel = lane_speed[target_lane];
            if (target_vel != SPEED_LIMIT) target_vel -= 0.1;
            
            // *** Generate trajectory ***
            // Add passed anchor points
            int prev_path_size = previous_path_x.size();
            double ref_x, ref_y, ref_yaw, ref_vel;
            vector<double> anchor_x, anchor_y;
            if (prev_path_size < 2) {                            // If no previous path add manual anchor points
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);
              ref_x = car_x;
              ref_y = car_y;
              ref_yaw = car_yaw;
              ref_vel = car_speed;
              anchor_x.push_back(prev_car_x);
              anchor_x.push_back(car_x);
              anchor_y.push_back(prev_car_y);
              anchor_y.push_back(car_y);
            } else {                                              // If there is a prev path use last two points as anchor
              ref_x = previous_path_x[prev_path_size - 1];
              ref_y = previous_path_y[prev_path_size - 1];
              double ref_x_prev = previous_path_x[prev_path_size - 2];
              double ref_y_prev = previous_path_y[prev_path_size - 2];
              ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
              
              double dx = ref_x - ref_x_prev;
              double dy = ref_y - ref_y_prev;
              ref_vel = sqrt(dx * dx + dy * dy) / DT;
              
              anchor_x.push_back(ref_x_prev);
              anchor_x.push_back(ref_x);
              anchor_y.push_back(ref_y_prev);
              anchor_y.push_back(ref_y);
            }
            
            // Add new anchor points
            double next_s, next_d;
            vector<double> next_frenet;
            
            next_s = fmod(car_s + 60.0, MAX_S);
            next_d = (0.05 * current_lane + 0.95 * target_lane) * 4.0 + 2.0;
            next_frenet = getXY(next_s, next_d);
            anchor_x.push_back(next_frenet[0]);
            anchor_y.push_back(next_frenet[1]);
            
            next_s = fmod(car_s + 90.0, MAX_S);
            next_d = target_lane * 4.0 + 2.0;
            next_frenet = getXY(next_s, next_d);
            anchor_x.push_back(next_frenet[0]);
            anchor_y.push_back(next_frenet[1]);
            
            next_s = fmod(car_s + 120.0, MAX_S);
            next_d = target_lane * 4.0 + 2.0;
            next_frenet = getXY(next_s, next_d);
            anchor_x.push_back(next_frenet[0]);
            anchor_y.push_back(next_frenet[1]);
            
            // Transform to car coordinates
            for (int i=0; i < anchor_x.size(); i++) {
              double shift_x = anchor_x[i] - ref_x;
              double shift_y = anchor_y[i] - ref_y;
              anchor_x[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
              anchor_y[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
            }
            
            // Create trajectory spline
            tk::spline s;
            s.set_points(anchor_x, anchor_y);
            
            // Push previous path to trajectory
            for (int i = 0; i < prev_path_size; i++) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }
            
            // *** Calculate new trajectory points ***
            double x = 0.0;
            double last_x = 0.0;
            double last_y = 0.0;
            double last_v = ref_vel;
            
            for (int i = 0; i < 50 - prev_path_size; i++) {
              
              if (ref_vel < target_vel) {
                ref_vel = min(target_vel, ref_vel + MAX_DV);
              } else {
                ref_vel = max(target_vel, ref_vel - MAX_DV);
              }
              x += (ref_vel * DT);
              
              double y;
              
              // Check if speed and acceleration limits are not breached
              while (true) {
                y = s(x);
                
                double dx = x - last_x;
                double dy = y - last_y;
                ref_vel = sqrt(dx * dx + dy * dy) / DT;
                double a = (ref_vel - last_v) / DT;

                if (ref_vel <= SPEED_LIMIT && a <= 10) break;
                
                x -= 0.005;
              }
              
              last_x = x;
              last_y = y;
              last_v = ref_vel;
              
              double rot_x = (x * cos(ref_yaw) - y * sin(ref_yaw));
              double rot_y = (x * sin(ref_yaw) + y * cos(ref_yaw));
              rot_x += ref_x;
              rot_y += ref_y;
              
              next_x_vals.push_back(rot_x);
              next_y_vals.push_back(rot_y);
            }
            
          }
          
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;
          
          auto msg = "42[\"control\","+ msgJson.dump()+"]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
















































































