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

#include "helper.hpp"
#include "car.hpp"
#include "position.hpp"

using namespace std;

// for convenience
using json = nlohmann::json;

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

Car car = Car();
//bool init = false;
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
    
//    map_waypoints_x.push_back(x);
//    map_waypoints_y.push_back(y);
//    map_waypoints_s.push_back(s);
//    map_waypoints_dx.push_back(d_x);
//    map_waypoints_dy.push_back(d_y);
    car.map_waypoints_x.push_back(x);
    car.map_waypoints_y.push_back(y);
    car.map_waypoints_s.push_back(s);
    car.map_waypoints_dx.push_back(d_x);
    car.map_waypoints_dy.push_back(d_y);
  }
  
  tk::spline spline_x, spline_y;
  spline_x.set_points(car.map_waypoints_s, car.map_waypoints_x);
  spline_y.set_points(car.map_waypoints_s, car.map_waypoints_y);
  car.map_waypoints_x.clear();
  car.map_waypoints_y.clear();
  car.map_waypoints_s.clear();
  
//  int spline_samples = 12000;
  for (double i = 0.0; i <= max_s; i+=0.5) {
    car.map_waypoints_x.push_back(spline_x(i));
    car.map_waypoints_y.push_back(spline_y(i));
    car.map_waypoints_s.push_back(i);
  }
  
//  h.onMessage([&car.map_waypoints_x,&car.map_waypoints_y,&car.map_waypoints_s,&car.map_waypoints_dx,&car.map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
//                                                                                                                           uWS::OpCode opCode) {
//    
  h.onMessage([](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          car_speed *= conv_mph2ms;

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
          
          if (cycle_counter < 2) {
//            car.best_trajectory.pos.clear();
            
//            Position p;
//            p.set_xy(2180, 1828);
//            car.best_trajectory.pos.push_back(p);
//            
//            Position p2;
//            p2.set_xy(2182, 1833);
//            car.best_trajectory.pos.push_back(p2);
            
          } else {
          

            
            car.best_trajectory.pos.clear();
            car.previous_trajectory.pos.clear();

            int prev_path_length = previous_path_x.size();
            cout << "Prev path length=" << prev_path_length << endl;
            
            Position start_pos;
            int no_points;
            if (prev_path_length == 0) {
              start_pos.set_xy(car_x, car_y);
              start_pos.set_theta(car_yaw);
              start_pos.set_v_xy(car_speed * cos(car_yaw), car_speed * sin(car_yaw));
              start_pos.set_a_xy(0.0, 0.0);          // we don't know, so we assume 0.0
              start_pos.set_v_total(car_speed);
              start_pos.calc_a_total();
              car.previous_trajectory.pos.push_back(start_pos);
              no_points = 200;
            } else {
//              int limit = min(100, prev_path_length);
              int limit = prev_path_length;
              no_points = 200 - prev_path_length;
              cout << "limit=" << limit << endl;
              Position prevPos;
              for (int i=0; i<limit; i++) {
                Position newPos;
                newPos.set_xy(previous_path_x[i], previous_path_y[i]);
                if (prevPos.is_xy_init()) {
                  newPos.calc_v_xy(prevPos);
                  newPos.calc_v_total();
                  if (prevPos.is_v_xy_init()) {
                    newPos.calc_a_xy(prevPos);
                    newPos.calc_a_total();
                  }
                }

                car.previous_trajectory.pos.push_back(newPos);
                prevPos = newPos;
              }
              
              start_pos = prevPos;
              start_pos.calc_theta(car.previous_trajectory.pos[limit - 2]);
              start_pos.calc_v_total();
            }
            
            cout << "NP:" << no_points << endl;
            if (no_points >= 25) {
              car.create_candidate_trajectories(start_pos, no_points);
            }
            //car.select_best_trajectory();
  
          }
          
//          cout << "OU" << endl;
          
          msgJson["next_x"] = car.get_best_trajectory_x();
          msgJson["next_y"] = car.get_best_trajectory_y();
          
          auto msg = "42[\"control\","+ msgJson.dump()+"]";
          
          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
          cout << endl;
          
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
















































































