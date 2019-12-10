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

  // start in lane 1
  // lane 0 is far left lane, lane 1 is middle, lane 2 is right
  int lane = 1;

  // setting ref_vel to 0 will get rid of the cold start problem going from
  // 0 to 49.5 instantly.
  double ref_vel = 0; // mph

  h.onMessage([&ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane]
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

          // Path Planning Code

          // 1. Prediction

          // set previous size, which comes from previous path size
          int prev_size = previous_path_x.size();

            // Sensor Fusion

          // look at the previous path size and if I have points to work with
          if(prev_size > 0)
          {
            car_s = end_path_s;
          }
          
          // Identify lane of other car
          bool too_close = false;
          bool car_left = false;
          bool car_right = false;

          // find ref_v to use
          for(int i = 0; i < sensor_fusion.size(); i++)
          {
              // car is in my lane
            float d = sensor_fusion[i][6];

            // Identify lane the other car is in
            int other_car_lane;
            if(d < 4 && d >= 0)
            {
              // other car is in leftmost lane
              other_car_lane = 0;
            }
            else if(d < 8 && d >= 4)
            {
              // other car is in middle lane
              other_car_lane = 1;
            }
            else if(d < 12 && d >= 8)
            {
              // other car is in rightmost lane
              other_car_lane = 2;
            }

            // pull speed info from vx and vy, the 3rd and 4th elements
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];

            // then check the speed, calculate the velocity magnitude of the other car
            double check_speed = sqrt(vx*vx + vy*vy);
            // pull the s value of the other car, know whether it's close or not
            double check_car_s = sensor_fusion[i][5];

            // if using the previous points, project s value outwards in time
            check_car_s += ((double)prev_size*0.02*check_speed);

            // Identify whether the other car is in our car's lane, the right or left lane
            // other car is in our car's lane
            if(other_car_lane == lane)
            {
              // if other car is in front of us and gap is smaller than 30m, our car is too close
              if( (check_car_s > car_s) && (check_car_s - car_s < 30) )
              {
                // our car is too close to the other car
                too_close = true;
              }
            }// other car is in right lane next to our car lane
            else if(other_car_lane - lane == 1)
            {
              if( ((car_s - 30) < check_car_s) && ((car_s + 30) > check_car_s) )
              {
                car_right = true;
              }
            } // other car is in left lane next to our car lane
            else if(lane - other_car_lane == 1)
            {
              if( ((car_s - 30) < check_car_s) && ((car_s + 30) > check_car_s) )
              {
                car_left = true;
              }
            }
          }

          // 2. Behavior Planning
          double acceleration = 0.224;
          double speed_limit = 49.5;

          // check if our car is too close to the other car ahead of us
          if(too_close)
          {
            // if another car isn't in right lane and our car isn't in rightmost lane
            if(car_right == false && lane < 2)
            {
              lane = lane + 1; // merge our car to the right lane
            } // if another car isn't in left lane and our car isn't in leftmost lane
            else if(car_left == false && lane > 0)
            {
              lane = lane - 1; // merge our car to the left lane
            } // nowhere to lane merge, decrease our car's acceleration
            else
            {
              // decreasing our car's acceleration by 0.224 ends up being 5 m/s^2
              // decrease our car's reference velocity by 5 meters per sec squared
              ref_vel = ref_vel - acceleration;
            }
          } // our car isn't too close to the other car ahead of us
          else
          {
            // if our car isn't in middle lane, merge back
            // if our car is in rightmost lane and another car isn't in left lane
            // or if our car is in leftmost lane and another car isn't in right lane
            if((lane == 2 && car_left == false) || (lane == 0 && car_right == false))
            {
              lane = 1; // merge our car to the middle lane
            }
            // if our car's reference velocity is less than the speed limit
            if(ref_vel < speed_limit)
            {
              // increase our car's acceleration, increasing its reference velocity
              ref_vel = ref_vel + acceleration;
            }
          }

          // 3. Trajectory Generation

          // Create a list of widely spaced (x, y) waypoints, evenly spaced at 30m
          // Later we will integpolate these waypoints with a spline and fill in with more points that control speed
          vector<double> ptsx;
          vector<double> ptsy;

          // reference x,y, yaw states
          // either we will reference the starting point as where the car is or at the previous paths end point
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // if previous size is almost empty, use the car as starting reference
          // that is if I am just starting out and don't have any previous path points
          if(prev_size < 2)
          {
            // Use two points that make the path tangent to the angle of the car
            // look at where the car is at and go back in time based on the angle
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            // generate two points to make sure that path is tangent
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          // use the previous path's endpoint as starting reference
          else
          {
            // what were the last couople of points that the car was following, then
            // calculating what angle the car was heading in using those last couple of points
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

            // Use two points that make the path tangent to the previous path's end point
            // Push them onto a list of previous points, 2 points on this pair of vectors
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);

          }

          // Use frenet, create 3 waypoints spaced 30 meters apart ahead of starting reference
          vector<double> next_wp0 = getXY(car_s+30,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);

          // add the location of our car in 30, 60, 90 meters to our vector of points
          // they already had 2 previous points, so now they have 5 points
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          // do a transformation to this local cars coordinates, we shift it so the car or that
          // last point of the previous path is at zero, the origin and its angle is at 0 degrees

          // we're taking it that the car is heading in x, y coord and want to take the car's
          // reference frame. so, it's just going straight ahead instead of being at 45 degrees

          for(int i = 0; i < ptsx.size(); ++i)
          {
            // shift car reference angle to 0 degrees
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = (shift_x * cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x * sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
          }

          // create a spline
          tk::spline s;

          // set points x,y to the spline
          // space these points along the spline, so our car goes desired speed
          s.set_points(ptsx,ptsy);

          // Define the actual (x,y) points we will use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Start with all previous path points from last time. If there are any
          // points from previous path, add them to path planner
          for(int i = 0; i < previous_path_x.size(); ++i)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Calculate how to break up spline points, so we travel at our desired reference velocity

          // our horizon going out 30m
          double target_x = 30.0;
          // ask spline what's the y for that given x
          double target_y = s(target_x);
          // we have target distance that is doing that distance calc from the car
          double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));
          // starts at 0, has to do with that local transformation we did, start at 0
          double x_add_on = 0;

          // Fill up rest of path planner with 50 points - points generated in previous path
          // Even if previous path generates 50 points, simulator may only go through 3 of those
          // points, so there are 47 points left over that will be reported next time around
          for(int i = 1; i <= 50-previous_path_x.size(); ++i)
          {
            double N = (target_dist/(0.02*ref_vel/2.24));

            // x point is each of those hash marks on the x axis of that visual
            // x add on is where we start, it's going to be 0, then we're adding
            // on our target x divided by N, which is the number of hash marks
            double x_point = x_add_on+(target_x)/N;

            // ask the spline what the y point is from the x point, so that tells us on the spline
            // where each little pearl or pac-man nugget or waypoint actually are, such that they 
            // are far enough apart that the car goes the right speed
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // rotate back to normal after rotating it earlier
            // we do the inverse of what we were doing before, we do a shift, then rotation
            x_point = (x_ref * cos(ref_yaw)-y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw)+y_ref * cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            // push that back to our next x and y vals and we're done!
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          json msgJson;
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