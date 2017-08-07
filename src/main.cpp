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
//#include "othercar.hpp"

using namespace std;
using namespace std::chrono;

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
  
  for (double i = 0.0; i <= max_s; i+=0.5) {
    car.map_waypoints_x.push_back(spline_x(i));
    car.map_waypoints_y.push_back(spline_y(i));
    car.map_waypoints_s.push_back(i);
  }
  
//  cout << "exp(3)=" << exp(3) << endl;
  
  /*
//  double min_di = numeric_limits<double>::max();
//  double min_s = 0;
//  double min_d = 0;
//  for (double s=124; s < 126; s += 0.01) {
//    for (double d=6.0; d < 6.2; d+=0.01) {
//      vector<double> f = getXY(s, d, car.map_waypoints_s, car.map_waypoints_x, car.map_waypoints_y);
//      double dx = f[0] - 909.48;
//      double dy = f[1] - 1128.67;
//      double dist = sqrt(dx * dx + dy * dy);
//      cout << "s=" << s << " / d=" << d << " / x=" << f[0] << " / y=" << f[1] << "                     *** dist" << dist << endl;
//      
//      if (dist < min_di) {
//        min_di = dist;
//        min_s = s;
//        min_d = d;
//      }
//    }
//  }
//  
//  cout << endl << "Min s=" << min_s << " / min_d=" << min_d << " / min dist=" << min_di << endl;
   */
  
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
        long long start_time = system_clock::now().time_since_epoch().count();
        
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
          
          car.current_lane = getLane(car_d);
          

          
          
        
          
          /*
          car.car_in_lane_dist = numeric_limits<double>::max();
          
          for (int i=0; i<sensor_fusion.size(); i++) {
            auto c = sensor_fusion[i];
            
            int id = c[0];
            double x = c[1];
            double y = c[2];
            double vx = c[3];
            double vy = c[4];
            double s = c[5];
            double d = c[6];
            
            if (getLane(d) == car.target_lane) {
              double dist_s = get_dist_s(car_s, s);
              
              if (dist_s < safe_dist && dist_s < car.car_in_lane_dist) {
                car.car_in_lane_dist = dist_s;
                OtherCar o(id, x, y, vx, vy, s, d, car.map_waypoints_s, car.map_waypoints_x, car.map_waypoints_y);
                
                if (o.v_total <= speed_limit) {
                  car.car_in_lane = o;
//                  cout << o.toString() << endl;
                }
              }
            }

          }
           */
          

          json msgJson;
          
          cycle_counter++;
          
          cout << "Cycle " << cycle_counter << ") Telemetry s=" << car_s << " / d=" << car_d << " / speed=" << car_speed
          << " / x=" << car_x << " / y=" << car_y << " / yaw=" << car_yaw << endl;
          
          vector<double> tx;
          vector<double> ty;
          
          if (cycle_counter >= 3 && previous_path_x.size() == 0) exit(1);
          
          if (cycle_counter < 2) {
            /*
//            car.best_trajectory.pos.clear();
            
//            Position p;
//            p.set_xy(2180, 1828);
//            car.best_trajectory.pos.push_back(p);
//            
//            Position p2;
//            p2.set_xy(2182, 1833);
//            car.best_trajectory.pos.push_back(p2);
            */
          } else {
            
            // Unpack sensor fusion
            car.change_left_blocked = false;
            car.change_right_blocked = false;
            if (car.target_lane == Lane::LEFT) {
              car.change_left_blocked = true;
            } else if (car.target_lane == Lane::RIGHT) {
              car.change_right_blocked = true;
            }
            car.target_lane_car_ahead_dist = numeric_limits<double>::max();
            car.target_left_lane_car_ahead_dist = numeric_limits<double>::max();
            car.target_right_lane_car_ahead_dist = numeric_limits<double>::max();
            
            cout << "Iterating sensor fusion: target_lane=" << car.target_lane << endl;
            cout << "change_left_blocked=" << car.change_left_blocked << " / change_right_blocked=" << car.change_right_blocked << endl;
            
            for (int i=0; i<sensor_fusion.size(); i++) {
              auto c = sensor_fusion[i];
              int id = c[0];
              double x = c[1];
              double y = c[2];
              double vx = c[3];
              double vy = c[4];
              double s = c[5];
              double d = c[6];
              
              if (d < -2.0 || d > 12) continue;
              
              OtherCar other = OtherCar(id, x, y, vx, vy, s, d, car.map_waypoints_s, car.map_waypoints_x, car.map_waypoints_y);
              cout << other.toString() << endl;
              
              bool occupies_left = (d <= 5.5);
              bool occupies_middle = ((d >= 2.5) && (d <= 9.5));
              bool occupies_right = (d >= 6.5);
              
              cout << "occupies left=" << occupies_left << " / middle=" << occupies_middle << " / right=" << occupies_right << endl;
              
              double ds = s - car_s;
              if (ds < -half_max_s) {
                ds += max_s;
              }
              if (ds > half_max_s) {
                ds -= max_s;
              }
              cout << "ds=" << ds << endl;
              
              bool is_in_blocking_dist = fabs(ds) < lane_change_safe_dist;
              
              if (car.target_lane == Lane::LEFT) {
                if (occupies_left && ds > 0 && ds < car.target_lane_car_ahead_dist) {
                  car.target_lane_car_ahead_dist = ds;
                  car.target_lane_car_ahead = other;
                }
                if (occupies_middle) {
                  if (is_in_blocking_dist) {
                    car.change_right_blocked = true;
                  } else if (ds > 0 && ds < car.target_right_lane_car_ahead_dist) {
                    car.target_right_lane_car_ahead_dist = ds;
                    car.target_right_lane_car_ahead = other;
                  }
                }
              }
              
              if (car.target_lane == Lane::MIDDLE) {
                if (occupies_left) {
                  if (is_in_blocking_dist) {
                    car.change_left_blocked = true;
                  } else if (ds > 0 && ds < car.target_left_lane_car_ahead_dist) {
                    car.target_left_lane_car_ahead_dist = ds;
                    car.target_left_lane_car_ahead = other;
                  }
                }
                if (occupies_middle && ds > 0 && ds < car.target_lane_car_ahead_dist) {
                  car.target_lane_car_ahead_dist = ds;
                  car.target_lane_car_ahead = other;
                }
                if (occupies_right) {
                  if (is_in_blocking_dist) {
                    car.change_right_blocked = true;
                  } else if (ds > 0 && ds < car.target_right_lane_car_ahead_dist) {
                    car.target_right_lane_car_ahead_dist = ds;
                    car.target_right_lane_car_ahead = other;
                  }
                }
              }
              
              if (car.target_lane == Lane::RIGHT) {
                if (occupies_right && ds > 0 && ds < car.target_lane_car_ahead_dist) {
                  car.target_lane_car_ahead_dist = ds;
                  car.target_lane_car_ahead = other;
                }
                if (occupies_middle) {
                  if (is_in_blocking_dist) {
                    car.change_left_blocked = true;
                  } else if (ds > 0 && ds < car.target_left_lane_car_ahead_dist) {
                    car.target_left_lane_car_ahead_dist = ds;
                    car.target_left_lane_car_ahead = other;
                  }
                }
              }
              
              cout << "change_left_blocked=" << car.change_left_blocked << " / change_right_blocked=" << car.change_right_blocked << endl;
            }
          
            car.best_trajectory.pos.clear();
            car.previous_trajectory.pos.clear();

            int prev_path_length = previous_path_x.size();
            
            Position start_pos;
            Position min_start_pos;
            int no_points;
            int min_no_points;
            
            if (prev_path_length == 0) {
              start_pos.set_xy(car_x, car_y);
              start_pos.set_theta(car_yaw);
//              start_pos.calc_sd(car.map_waypoints_x, car.map_waypoints_y);
              
              if (cycle_counter > 2) {
                start_pos.s = car_s;
                start_pos.d = car_d;
              } else {  // with the provided starting coordinates the start is jerky
                start_pos.s = 124.93;
                start_pos.d = 6.1;
              }
              
              start_pos.set_v_xy(car_speed * cos(car_yaw), car_speed * sin(car_yaw));
              start_pos.set_a_xy(0.0, 0.0);          // we don't know, so we assume 0.0
              start_pos.set_v_total(car_speed);
              start_pos.calc_a_total();
              
              min_start_pos = start_pos;
              
              car.previous_trajectory.pos.push_back(start_pos);
              no_points = desired_path_len;
              min_no_points = desired_path_len;
              
              car.current_lane = getLane(car_d);
            } else {
              int limit = min(desired_path_len, prev_path_length);
              no_points = desired_path_len - limit;
              
              Position prevPos;
              for (int i=0; i<limit; i++) {
                Position newPos;
                newPos.set_xy(previous_path_x[i], previous_path_y[i]);
                newPos.calc_v_xy(prevPos);
                newPos.calc_v_total();
                
//                newPos.calc_a_xy(prevPos);
//                newPos.calc_a_total();
             
                car.previous_trajectory.pos.push_back(newPos);
                prevPos = newPos;
              }
              
              start_pos = prevPos;
              start_pos.calc_theta(car.previous_trajectory.pos[limit - 2]);
              start_pos.calc_v_theta();
              start_pos.calc_sd(car.map_waypoints_x, car.map_waypoints_y);
              
              if (car.previous_trajectory.pos.size() >= 25) {
                min_start_pos = car.previous_trajectory.pos[24];
                min_no_points = 175;
                min_start_pos.calc_theta(car.previous_trajectory.pos[23]);
                min_start_pos.calc_v_theta();
                min_start_pos.calc_sd(car.map_waypoints_x, car.map_waypoints_y);
              } else {
                min_start_pos = start_pos;
                min_no_points = no_points;
              }
            
            }
            
//            cout << "NP:" << no_points << endl;

            
//            if (no_points >= 25) {
//              car.create_candidate_trajectories(start_pos, no_points);
//              tx = car.get_best_trajectory_x();
//              ty = car.get_best_trajectory_y();
//            } else {
//              for (int i = 0; i<previous_path_x.size(); i++) {
//                tx.push_back(previous_path_x[i]);
//                ty.push_back(previous_path_y[i]);
//              }
//            }
            
            car.create_candidate_trajectories(start_pos, no_points, min_start_pos, min_no_points, start_time);
          }
          
          tx = car.get_best_trajectory_x();
          ty = car.get_best_trajectory_y();
          
          
//          for (int i=0; i < tx.size(); i++) {
//            cout << "x=" << tx[i] << " / y=" << ty[i] << endl;
//          }
          
          msgJson["next_x"] = tx;
          msgJson["next_y"] = ty;
          
          auto msg = "42[\"control\","+ msgJson.dump()+"]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
          long long end_time = system_clock::now().time_since_epoch().count();
//          cout << "Cycle took " << (end_time - start_time) << "ms." << endl << endl;
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
















































































