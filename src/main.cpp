#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.
  
  std::vector<double> p = {0, 0, 0};//(3, 0.0);
  std::vector<double> dp = {1, 10, 0.001};
  std::vector<bool> p_tune_attempt(3, false);
  std::vector<bool> p_tune_attempt2(3, false);
  double tolerance = 0.000000001;

  pid.Init(p[0], p[1], p[2]);


  double best_err = INFINITY;

  int tune_idx = 0;

  h.onMessage([&pid, &p, &dp, &p_tune_attempt, &p_tune_attempt2, &tune_idx, &tolerance, &best_err](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          double sum_dp = 0.0;

          std::for_each(dp.begin(), dp.end(), [&] (double n) {
              sum_dp += n;
          });

          std::cout << "sum_dp = " << sum_dp << std::endl;

          bool tune = sum_dp > tolerance;

          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;

          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          double totalError = 0.0;

          double err = cte*cte;

          if (tune) {
            auto i = tune_idx%dp.size();
            tune_idx = i;

            std::cout << "tune_idx = " << tune_idx << std::endl;
            
            if (p_tune_attempt[i]) {
              if (err < best_err) {
                best_err = err;
                dp[i] *= 1.1;
                ++tune_idx;
              }
              else {
                p[i] -= 2*dp[i];
                //pid.Init(p[0], p[1], p[2]);
                p_tune_attempt2[i] = true;
              }
              
              //pid.UpdateError(cte);
              //totalError = pid.TotalError();
              p_tune_attempt[i] = false;
            }
            else if (p_tune_attempt2[i]) {
              if (err < best_err) {
                best_err = err;
                dp[i] *= 1.1;
                
              } 
              else {
                p[i] += dp[i];
                dp[i] *= 0.9;
                //pid.Init(p[0], p[1], p[2]);
              }

              //pid.UpdateError(cte);
              //totalError = pid.TotalError();

              ++tune_idx;
              p_tune_attempt2[i] = false;
            }
            else {
              p[i] += dp[i];
              
              p_tune_attempt[i] = true;

              if (best_err == INFINITY) {
                best_err = err;
              }
            }

            pid.Init(p[0], p[1], p[2]);

            std::cout << "dp[0]=" << dp[0] << "; dp[1]=" << dp[1] << "; dp[2]=" << dp[2] << std::endl;
            std::cout << "p[0]=" << p[0] << "; p[1]=" << p[1] << "; p[2]=" << p[2] << std::endl;
          }
          else {
            std::cout << "tuning completed" << std::endl;
            //pid.UpdateError(cte);
            //totalError = pid.TotalError();
            
          }

          pid.UpdateError(cte);
          totalError = pid.TotalError();


          steer_value = -totalError;

          /*if (fabs(steer_value) > 1.0) {
            steer_value = steer_value < 0.0 ? -1 : 1;
          }*/
          
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
