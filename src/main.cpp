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
using Eigen::MatrixXd;

/*
vector<double> JMT(vector<double> &start, vector<double> &end, double T) {
  /**
   * Calculate the Jerk Minimizing Trajectory that connects the initial state
   * to the final state in time T.
   *
   * @param start - the vehicles start location given as a length three array
   *   corresponding to initial values of [s, s_dot, s_double_dot]
   * @param end - the desired end state for vehicle. Like "start" this is a
   *   length three array.
   * @param T - The duration, in seconds, over which this maneuver should occur.
   *
   * @output an array of length 6, each value corresponding to a coefficent in 
   *   the polynomial:
   *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
   *
   * EXAMPLE
   *   > JMT([0, 10, 0], [10, 10, 0], 1)
   *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   

  MatrixXd A = MatrixXd(3, 3);
  A << T*T*T, T*T*T*T, T*T*T*T*T,
       3*T*T, 4*T*T*T,5*T*T*T*T,
       6*T, 12*T*T, 20*T*T*T;
    
  MatrixXd B = MatrixXd(3,1);     
  B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
       end[1]-(start[1]+start[2]*T),
       end[2]-start[2];
          
  MatrixXd Ai = A.inverse();
  
  MatrixXd C = Ai*B;
  
  vector <double> result = {start[0], start[1], .5*start[2]};

  for(int i = 0; i < C.size(); ++i) {
    result.push_back(C.data()[i]);
  }

  return result;
}
*/

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

  // defining hyper-parameters
  struct hyper_params{
    int NUM_POINTS; // maximum number of the points to be planned by the trajectory planner
    double dt;  // The time increment of the planned points in seconds
    double lane_width;  // lane width in m
    double safe_s;  // safe s distance in front of the car
    double max_vel; // maximum allowable speed mph
    double inc; // increment in Frenet s 
    double mph2mps; // miles per hour to meters per second
  } par;
  par.NUM_POINTS = 40; 
  par.dt = 0.02; 
  par.lane_width = 4; 
  par.safe_s = 30.0; 
  par.max_vel = 48; 
  par.inc = 30; 
  par.mph2mps = 0.447; 

  double ref_vel = 0; // mph
  int lane_idx = 1; // lane index: 0,1,2
  
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,
               &par, &ref_vel, &lane_idx]
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
          int prev_size = previous_path_x.size();

          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          // flag if our vehicle is too close to the one in front
          bool too_close = false;
          bool too_close_left = false;
          bool too_close_right = false;
          if (lane_idx == 0)  too_close_left = true;
          if (lane_idx == 2)  too_close_right = true;
          // find reference velocity to use
          for (int i = 0; i < sensor_fusion.size(); ++i) {
            // check if the other car is in our lane
            double d = sensor_fusion[i][6];
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx + vy*vy);
            double check_s_current = sensor_fusion[i][5];
            // finding out where that car is going to be if all the current path points were to be covered by our car
            double check_s_future = check_s_current + prev_size * par.dt * check_speed;

            // if the other car is in our lane
            if ( d < (par.lane_width * (lane_idx+1)) &&  d > (par.lane_width * (lane_idx)) ){
              /*
              * The if check a few conditions:
              * 1. Whether or not the vehicle we are checking for is in front of the car
              * 2. In the future (when our vehicle is done completing the planned points), whether the vehicle we are checking for is within 
              *    the safe distance of our vehicle
              */
              if ( (check_s_future - end_path_s) < par.safe_s && check_s_current > car_s ){
                // do some logic: lower reference velocity or flage to try to change lane
                too_close = true;
              }
            }
            // if the other car is in our left lane
            if ( d < (par.lane_width * (lane_idx)) &&  d > (par.lane_width * (lane_idx-1)) && lane_idx != 0 ){
              /*
              * The if check a few conditions:
              * 1. Whether or not the vehicle we are checking for is in front of the car
              * 2. In the future (when our vehicle is done completing the planned points), whether the vehicle we are checking for is within 
              *    the safe distance of our vehicle
              */
              if ( (check_s_future - end_path_s) < par.safe_s && check_s_current > car_s ){
                // do some logic: lower reference velocity or flage to try to change lane
                too_close_left = true;
              }
            }
            // if the other car is in our right lane
            if ( d < (par.lane_width * (lane_idx+2)) &&  d > (par.lane_width * (lane_idx+1)) && lane_idx != 2 ){
              /*
              * The if check a few conditions:
              * 1. Whether or not the vehicle we are checking for is in front of the car
              * 2. In the future (when our vehicle is done completing the planned points), whether the vehicle we are checking for is within 
              *    the safe distance of our vehicle
              */
              if ( (check_s_future - end_path_s) < par.safe_s && check_s_current > car_s ){
                // do some logic: lower reference velocity or flage to try to change lane
                too_close_right = true;
              }
            }   
          }
          // 10 m/s^2 is accel/decel limit
          if (too_close){
            if (ref_vel > 0.1){
              ref_vel -= 0.1 / par.mph2mps; // slow down 0.1 m/s
            }
            if (!too_close_left){
              lane_idx -= 1;
            }
            else if (!too_close_right){
              lane_idx += 1;
            }
          } 
          else if (ref_vel < par.max_vel){
            ref_vel += 0.1 / par.mph2mps;
          }


          // create a list of widely spaced (x,y) waypoints, evenly spaced at 30 m
          vector<double> ptsx;
          vector<double> ptsy;
          vector<double> ptss;

          // reference x,y,yaw
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_s = car_s;
          double ref_yaw = deg2rad(car_yaw);

          // if previous path is almost empty, use the car as the starting reference
         
          if (prev_size<2){
            // use two points that make the path tangent to the car
            double prev_car_x = ref_x - cos(ref_yaw);
            double prev_car_y = ref_y - sin(ref_yaw);
            double prev_car_s = ref_s - distance(ref_x, ref_y, prev_car_x, prev_car_y);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(ref_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(ref_y);

            ptss.push_back(prev_car_s);
            ptss.push_back(ref_s);
          }
          // use the previous path's end point as the starting reference
          else{
            // redefine reference state as previous path end point
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];
            ref_s = end_path_s;

            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            double ref_s_prev = ref_s - distance(ref_x, ref_y, ref_x_prev, ref_y_prev);
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            // use two points that makes path tangent to previous path's end point
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);

            ptss.push_back(ref_s_prev);
            ptss.push_back(ref_s);
          }

          // in Frenet add evenly spaced points in front of the reference point
          auto next_wp0 = getXY(ref_s + par.inc,   (0.5+lane_idx)*par.lane_width, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          auto next_wp1 = getXY(ref_s + 2*par.inc, (0.5+lane_idx)*par.lane_width, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          auto next_wp2 = getXY(ref_s + 3*par.inc, (0.5+lane_idx)*par.lane_width, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptss.push_back(ref_s + par.inc);
          ptss.push_back(ref_s + 2*par.inc);
          ptss.push_back(ref_s + 3*par.inc);
          
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          // create spline
          tk::spline sx;
          tk::spline sy;

          // set x y points of the spline
          sx.set_points(ptss,ptsx);
          sy.set_points(ptss,ptsy);

          // define the actual (x,y) points to be passed to planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          // start from all of the previous path points
          for (int i = 0; i < prev_size; ++i){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          // increment in s to get to almost reference velocity
          double ds = (ref_vel * par.mph2mps * par.dt);

          // fill out the rest of path planner after filling it with previous points 
          for (int i = 0; i < par.NUM_POINTS - prev_size; ++i){
            double x_point = sx(ref_s + (i+1)*ds);
            double y_point = sy(ref_s + (i+1)*ds);

            // appending to the points
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