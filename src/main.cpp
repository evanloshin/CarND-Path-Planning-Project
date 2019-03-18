#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

//set car's initial state
double ref_vel = 0;
int curr_lane = 1;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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
  
  
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          //set target vehicle control params
          double SPEED_LIMIT = 49 / 2.24; //MPH->m/s
          string STATE = "KL";
          double target_speed = SPEED_LIMIT;
          if(car_d <= 4)
          {
            curr_lane = 0;
          }
          else if(car_d < 8)
          {
            curr_lane = 1;
          }
          else
          {
            curr_lane = 2;
          }
          int TARGET_LANE = curr_lane;

          double delta_t = 0.02;
          int prev_size = previous_path_x.size();

          //initialize collision avoidance
          bool collision = false;
          double collision_buffer = 20; //meters
          double approach_buffer = 30; //meters
          if(prev_size > 0)
          {
            car_s = end_path_s;
          }

          //check cars in sensor fusion for possible collision
          for(int i=0; i<sensor_fusion.size(); i++)
          {
            int sensor_d = sensor_fusion[i][6];
            if(sensor_d < 4 + curr_lane * 4 && sensor_d > curr_lane * 4)
            {
              double sensor_s = sensor_fusion[i][5];
              double sensor_vx = sensor_fusion[i][3];
              double sensor_vy = sensor_fusion[i][4];
              double sensor_v = sqrt(pow(sensor_vx, 2) + pow(sensor_vy, 2));

              double sensor_s_predicted = sensor_s + prev_size * sensor_v * delta_t;
              if(car_s < sensor_s_predicted && sensor_s_predicted - car_s < collision_buffer)
              {
                target_speed = sensor_v - 0.2;
                collision = true;
              }
              else if(car_s < sensor_s_predicted && sensor_s_predicted - car_s < approach_buffer)
              {
                collision = true;
              }
            }
          }
          
          //check speed of adjacient lanes
          vector<int> lcl_cars;
          vector<int> lcr_cars;
          double lcl_speed = SPEED_LIMIT;
          double lcr_speed = SPEED_LIMIT;
          double min_passing_dist = 20;
          double min_rear_dist = 7;
          for(int i=0; i<sensor_fusion.size(); i++)
          {
            int sensor_d = sensor_fusion[i][6];
            double sensor_s = sensor_fusion[i][5];
            double sensor_vx = sensor_fusion[i][3];
            double sensor_vy = sensor_fusion[i][4];
            double sensor_v = sqrt(pow(sensor_vx, 2) + pow(sensor_vy, 2));
            double sensor_s_predicted = sensor_s + prev_size * sensor_v * delta_t;
            //calc speed of lane to the left
            if(sensor_d < curr_lane * 4)
            {
              if(sensor_s_predicted > car_s && sensor_s_predicted - car_s < collision_buffer)
              {
                lcl_cars.push_back(i);
              }
              else if(sensor_s_predicted < car_s && car_s - sensor_s_predicted < min_rear_dist)
              {
                lcl_cars.push_back(i);
              }
              if(sensor_s_predicted > car_s && sensor_s_predicted - car_s < min_passing_dist && sensor_v < lcl_speed)
              {
                lcl_speed = sensor_v;
              }
            }
            //calc speed of lane to the right
            else if(sensor_d > 4 + curr_lane * 4)
            {
              if(sensor_s_predicted > car_s && sensor_s_predicted - car_s < collision_buffer)
              {
                lcr_cars.push_back(i);
              }
              else if(sensor_s_predicted < car_s && car_s - sensor_s_predicted < min_rear_dist)
              {
                lcr_cars.push_back(i);
              }
              if(sensor_s_predicted > car_s && sensor_s_predicted - car_s < min_passing_dist && sensor_v < lcr_speed)
              {
                lcr_speed = sensor_v;
              }
            }
          }
          //select next state with preference to passing on the left
          if(curr_lane > 0 && collision == true && lcl_speed > target_speed + 1.25)
          {
            STATE = "PLCL";
          }
          else if(curr_lane < 2 && (lcr_speed >= target_speed + 1.25 || lcr_cars.size() == 0))
          {
            //check if car is still getting up to speed in lane
            if(collision == false && car_speed < target_speed - 1.5)
            {
              STATE = "KL";
            }
            else
            {
              STATE = "PLCR";
            }
          }
          
          
          //slow car down if collision detected
          if(ref_vel > target_speed)
          {
            ref_vel -= 0.22; //m/s
          }
          else if(ref_vel < target_speed)
          {
            ref_vel += 0.22;
          }
          
          //implement PLCL state
          if(STATE == "PLCL")
          {
            //check for collision in adjacient lane
            if(lcl_cars.size() == 0)
            {
              STATE = "LCL";
              TARGET_LANE -= 1;
            }
          }
          else if(STATE == "PLCR")
          {
            if(lcr_cars.size() == 0)
            {
              STATE = "LCR";
              TARGET_LANE += 1;
            }
          }
          
          //create list of sparsely spaced waypoints for spline interpolation
          vector<double> sparse_pts_x;
          vector<double> sparse_pts_y;

          //store car state after each new point in the path
          double ref_x = car_x;
          double ref_y = car_y;
          double prev_x;
          double prev_y;
          double ref_yaw = deg2rad(car_yaw);

          //use car's tangential position as starting point if previous path is empty
          if (prev_size < 2)
          {
            prev_x = car_x - cos(car_yaw);
            prev_y = car_y - sin(car_yaw);

            sparse_pts_x.push_back(prev_x);
            sparse_pts_x.push_back(car_x);
            sparse_pts_y.push_back(prev_y);
            sparse_pts_y.push_back(car_y);
          }
          //otherwise use the previous path's points
          else
          {
            prev_x = previous_path_x[prev_size - 2];
            prev_y = previous_path_y[prev_size - 2];
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];
            ref_yaw = atan2(ref_y - prev_y, ref_x - prev_x);
            sparse_pts_x.push_back(prev_x);
            sparse_pts_x.push_back(ref_x);
            sparse_pts_y.push_back(prev_y);
            sparse_pts_y.push_back(ref_y);
          }

          //calculate waypoints at 30m increments
          for(int i=1; i<=3; i++)
          {
            double wpnt_s = car_s + i * 30;
            double wpnt_d = 2 + TARGET_LANE * 4;
            vector<double> wpnt_xy = getXY(wpnt_s, wpnt_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            sparse_pts_x.push_back(wpnt_xy[0]);
            sparse_pts_y.push_back(wpnt_xy[1]);
          }

          //shift points to car's origin
          for(int i=0; i < sparse_pts_x.size(); i++)
          {
            double shift_x = sparse_pts_x[i] - ref_x;
            double shift_y = sparse_pts_y[i] - ref_y;
            
            sparse_pts_x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0-ref_yaw);
            sparse_pts_y[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0-ref_yaw);
          }
          
          //define a spline
          tk::spline spl;
          spl.set_points(sparse_pts_x, sparse_pts_y);

          //use any previous path points
          for(int i=0; i<prev_size; i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          //distribute points along spline according to desired velocity
          double target_x = 30; //meters
          double target_y = spl(target_x);
          double linear_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));
          double x_start = 0;
          for(int i=1; i<=50-prev_size; i++)
          {
            //calculate next point along spline
            double num_pts = linear_dist / (delta_t * ref_vel);
            double linear_inc = target_x / num_pts;
            double traj_x = x_start + linear_inc;
            double traj_y = spl(traj_x);

            //update current position along spline for next iteration
            x_start = traj_x;

            //transform point back to global coordinate
            double x_ref = traj_x;
            double y_ref = traj_y;
            traj_x = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            traj_y = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
            traj_x += ref_x;
            traj_y += ref_y;

            //add coordinate to path
            next_x_vals.push_back(traj_x);
            next_y_vals.push_back(traj_y);
          }
          
          //END TODO
          
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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
