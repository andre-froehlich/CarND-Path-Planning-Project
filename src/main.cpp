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
const double SPEED_LIMIT = 22.1; //22.3
const double MAX_DV = 0.15;

// convert degrees to radians
double deg2rad(double x) { return x * M_PI / 180; }

// convert frenet to xy coordinates
tk::spline spline_wp_x;
tk::spline spline_wp_y;
tk::spline spline_wp_dx;
tk::spline spline_wp_dy;
vector<double> getXY(double s, double d) {
  //  cout << "x="<< spline_wp_x(124.5) << " / y=" << spline_wp_y(124.5) << " / dx=" << spline_wp_dx(124.5) << " / dy=" << spline_wp_dy(124.5) << endl;
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
//        long long start_time = system_clock::now().time_since_epoch().count();
        
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
          
          cout << "Cycle " << cycle_counter << ") Telemetry s=" << car_s << " / d=" << car_d << " / speed=" << car_speed
          << " / x=" << car_x << " / y=" << car_y << " / yaw=" << car_yaw << endl;
          
          vector<double> next_x_vals, next_y_vals;
          
          
          if (cycle_counter < 2) {
            
          } else {
            // Unpack sensor fusion
            // Decide strategy
            
            int prev_path_size = previous_path_x.size();
            
            double ref_x, ref_y, ref_yaw, ref_vel;
//            ref_vel = min(car_speed + 1.0, SPEED_LIMIT);
            
            // Add passed anchor points
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
              
            } else {                                        // If there is a prev path use last two points as anchor
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
//              last_point = traj[traj.size() - 1];
            }
            
            // Add new anchor points
            double next_s, next_d;
            vector<double> next_frenet;
            
            next_s = fmod(car_s + 30.0, MAX_S);
            next_d = 6.0;
            next_frenet = getXY(next_s, next_d);
            anchor_x.push_back(next_frenet[0]);
            anchor_y.push_back(next_frenet[1]);
            
            next_s = fmod(car_s + 60.0, MAX_S);
            next_d = 6.0;
            next_frenet = getXY(next_s, next_d);
            anchor_x.push_back(next_frenet[0]);
            anchor_y.push_back(next_frenet[1]);
            
            next_s = fmod(car_s + 90.0, MAX_S);
            next_d = 6.0;
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
            
            
            for (int i = 0; i < prev_path_size; i++) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }
            
            double target_vel = SPEED_LIMIT;
            double x = 0.0;
            
            for (int i = 0; i < 50 - prev_path_size; i++) {
              if (ref_vel < target_vel) {
                ref_vel = min(target_vel, ref_vel + MAX_DV);
              } else {
                ref_vel = max(target_vel, ref_vel - MAX_DV);
              }
      
              x += (ref_vel * DT);
              
              double y = s(x);
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
















































































