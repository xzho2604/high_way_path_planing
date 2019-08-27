#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
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

  //----------------------------------------------------
  // add my variables: lane 0 far left lane ,lane 1 is middle , lane 2 is right lane
  int lane = 1;
  // reference volocity
  double ref_vel= 0;

  //----------------------------------------------------
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&ref_vel, &lane]
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
          // the last path the car follows
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road. is a vector of vector of double
          auto sensor_fusion = j[1]["sensor_fusion"];


         //----------------------------------------------
         int prev_size = previous_path_x.size();

         //----------------------------------------------
         // sensor_fusion
         // check the sensor_fusion list of other cars around us
         // get their current state and decide what we should behave to avoid collision

         // if there is previous points use the last point prevous path
         auto curr_car_s = car_s; // copy of current car s not furture
         if(prev_size > 0) car_s = end_path_s;// set car state into the future end of previous planing

         // define some constants
         const double BUF_DIS = 30; // buffer distance to keep with car in the front
         const double MAX_SPEED = 49.5; // speed limit
         const double SPEED_DEL = 0.224; // speed increase each time
         const double LANE_WIDTH = 4; // speed increase each time
         const double NUM_LANE = 3; // speed increase each time
         
         // some flags for control
         bool front_close = false; // car infront too close
         bool left_car = false; // left lane clear to go
         bool right_car = false; // clear to go right

         // loop through all cars detected on the same side of the road
         for(int i = 0; i<sensor_fusion.size();++i){
             // get the d value from each other car
             float d = sensor_fusion[i][6]; // d value of the ith car in sensor_fusion
             if(d < 0 || d > NUM_LANE*LANE_WIDTH) continue; // car outside lane not considered
 
             // we need to check the speed of the car in our lane
             double vx = sensor_fusion[i][3];
             double vy = sensor_fusion[i][4];
             double check_speed = sqrt(vx*vx + vy*vy); // compute the car speed magnitude
             double check_car_s = sensor_fusion[i][5];

             // if((check_car_s - car_s_copy) < 100 && (check_car_s - car_s_copy) > 0 )
             //   follow_speed  = check_speed; 

             // if we use the previous unexcuted path planing point we can
             // project where the other cars are at the end of the planing planin 
             // using previous path planing
             auto curr_check_car_s = check_car_s; // current location of ther car
             check_car_s += static_cast<double>(prev_size)*0.02*check_speed;
             
             // know where other cars detected are in which lane
             int other_car_lane = floor(d/4); // since lane is 4 metres wide

             // if the car is in the same lane
             // check if other car is ahdead of us and distance is less than a saftye margine
             if(other_car_lane == lane) { 
                 if(check_car_s > car_s && check_car_s - car_s < BUF_DIS){
                     front_close = true;
                }
             }
           
            // if other car is in our left lane 
            // check if that car is obscure our safty range to change lane
            if(other_car_lane - lane == -1) {
                if(car_s - BUF_DIS < check_car_s && car_s + BUF_DIS > check_car_s) {
                    // && curr_car_s - 10 < curr_check_car_s && curr_car_s + 10 > curr_check_car_s){
                     left_car = true;
                }
            }

            // if other car is in our right lane 
            if(other_car_lane - lane == 1) {
               if(car_s - BUF_DIS < check_car_s && car_s + BUF_DIS > check_car_s){
                 //  && curr_car_s - 10 < curr_check_car_s && curr_car_s + 10 > curr_check_car_s){
                    right_car = true;
               }
            }
         }

         // by the end of the previous path planing if the other
         // car is ahead of us and less thant 30 metres with our car
         // we need to slow down to avoid collision
         if(front_close){
             // left lane clear and our car not in left most lane change to left lane
             if(!left_car && lane > 0){ 
                --lane;
             // else if righ lane clear and our car not in the right most lane
             }else if(!right_car && lane != 2) {
                 ++lane;
             // can not go either righ or left keep in the lane and 
             // reduce speed to follow the car in the front
             } else { 
                ref_vel -= SPEED_DEL; // about 5 metres per second
             }

         } else {
             // if we are currently not in the centre lane but was able to move to 
             // the centre lane either from right or left lane , switch to the centre
             if(lane != 1 && (( lane == 0 && !right_car ) || ( lane == 2 && !left_car ))){
                 lane = 1;
             }
             
             // increase speed if below speed limit
             if(ref_vel < MAX_SPEED)
                 ref_vel += SPEED_DEL;
         }

         // end sensor_fusion 
         //----------------------------------------------

         // create a sparse waypoints for later spline fit
         vector<double> ptsx;
         vector<double> ptsy;

         // reference x, y ,yaw status
         // reference as car's inital state for this path planinng
         double ref_x = car_x;
         double ref_y = car_y;
         double ref_yaw = deg2rad(car_yaw);

         // check if previous path is empty or not
         // use the last couple of points from the previous path and 
         // find out where the car is heading
         if(prev_size <2 ){
             // use current car state to induce it previous state
             // based on where currently the car is heading
             double prev_car_x = car_x - cos(car_yaw);
             double prev_car_y = car_y - sin(car_yaw);

             ptsx.push_back(prev_car_x);
             ptsx.push_back(car_x);

             ptsy.push_back(prev_car_y);
             ptsy.push_back(car_y);

         } else { // we will just use the last two points in the previosu path
             ref_x = previous_path_x[prev_size - 1]; // last point in previous path
             ref_y = previous_path_y[prev_size - 1];

             double ref_x_prev = previous_path_x[prev_size -2]; // second last point
             double ref_y_prev = previous_path_y[prev_size -2];
             ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);// induce angle car is heading

             ptsx.push_back(ref_x_prev);
             ptsx.push_back(ref_x);

             ptsy.push_back(ref_y_prev);
             ptsy.push_back(ref_y);
         }

         // in Frenet Coordinates add another 3 points that are 30 metres apart to the 
         // starting pont
         vector<double> next_wp0 = getXY(car_s + 30, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y); 
         vector<double> next_wp1 = getXY(car_s + 60, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y); 
         vector<double> next_wp2 = getXY(car_s + 90, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y); 

         ptsx.push_back(next_wp0[0]);
         ptsx.push_back(next_wp1[0]);
         ptsx.push_back(next_wp2[0]);
         
         ptsy.push_back(next_wp0[1]);
         ptsy.push_back(next_wp1[1]);
         ptsy.push_back(next_wp2[1]);

         // now in ptsx and ptsy we have two points from the previous path and 
         // 3 points following the lane for this path plan that are evenly sparsely apart 30 metres
         // so total of 5 points now
         // now do a shift and rotation to the car's coordinates system
         for(int i =0;i < ptsx.size(); ++i){
             // make the starting point in the car coordinates 0 0
             double shift_x = ptsx[i] - ref_x;
             double shift_y = ptsy[i] - ref_y;
             
             // adjust heading so that the inital heading in the car coordinates is 0
             ptsx[i] = (shift_x*cos(0 - ref_yaw)-shift_y*sin(0-ref_yaw));
             ptsy[i] = (shift_x*sin(0 - ref_yaw)+shift_y*cos(0-ref_yaw));

         }

         tk::spline s; // create a spline
         //std::cout <<"here here :"<< ptsx.size() << " "<< ptsy.size() << std::endl;
         s.set_points(ptsx,ptsy); // set (x,y) points to the spline

         // ----------------------------------------------------------
          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // add anothing from the previous path if there is 
          // these are the points that are planned previously but
          // not being excuted , so we can use them for this path planing
          // instead of starting from stratch
          // to help out with the smooth transition
          for(int i =0; i< previous_path_x.size();++i){
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
          }
          
          // calculate how to break up the slline so that we can travel at the 
          // desired speed
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x)*(target_x) + (target_y)*(target_y));

          double x_add_on = 0;

          // fill in the rest of the car planinig points after we added the previous 
          // unexcuted planinng points so that each path planining is always 50 points
          for(int i = 1; i<= (50 -previous_path_x.size());++i){
                
              // / 2.24 to translate ref_vel into metres per s
              double N = (target_dist/(0.02*ref_vel/2.24)); // find what N needs to be  
              double x_point = x_add_on+(target_x)/N; // evenly spaced x coordinates of n such
              double y_point = s(x_point); // pass x to the spline function to find out y

              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              // rotate the points to the normal coordiantes system that 
              // we have translated to car coordiantes previously
              // we have translated to car coordiantes previously
              x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
              y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);

          }
          // ==========================================================================
          // TODO: end

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
