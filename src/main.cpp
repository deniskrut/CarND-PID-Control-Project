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
  
  // Current twiddle simulation run
  int twiddle_count = 0;
  // Total steps for one simulation
  int twiddle_one_step_count = 5000;
  // Best error known
  double best_error = INFINITY;
  // Index of variable currently being optimized
  int cur_variable = 0;
  // Current direction of variable optimization. 1 - up, -1 - down.
  int cur_direction = 1;
  // Variables for PID
  double p_vars[3] = {0.1, 0., 10.};
  // Delta for variables for PID
  double dp_vars[3] = {0.5, 1.93633e-05, 1.};
  // Starter value for twiddle
  pid.Init(p_vars[0], p_vars[1], p_vars[2]);
  // Error calculation for Twiddle
  double twiddle_error = 0;
  // Throttle value for twiddle
  double throttle = 0.484;
  // Indicates if twiddle is enabled
  bool twiddle_enabled = false;
  // Was the track completed?
  bool track_completed = true;
  // PID for throttle
  PID pid_throttle;
  // Init PID for throttle
  pid_throttle.Init(.1, 0., 1.);
  // Previous step time, needed for more presize speed control
  std::chrono::high_resolution_clock::time_point last_time;
  
  h.onMessage([&](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          
          // If we made less then desired number of steps, continue simulation
          if (twiddle_count < twiddle_one_step_count || !twiddle_enabled)
          {
            // j[1] is the data JSON object
            double cte = std::stod(j[1]["cte"].get<std::string>());
            double speed = std::stod(j[1]["speed"].get<std::string>());
            double angle = std::stod(j[1]["steering_angle"].get<std::string>());
            double steer_value;
          
            // Update error for PID
            pid.UpdateError(cte);
            // Calculate steering angle
            steer_value = -pid.TotalError();
            
            // Calculate tame elapsed since last message
            auto cur_time = std::chrono::high_resolution_clock::now();
            double dt = std::chrono::duration_cast<std::chrono::duration<double>>(cur_time - last_time).count();
            last_time = cur_time;
            
            // The more time have elapsed since last update, the more cautious we should be
            // Therefore we should reduce our speed approaching barriers more agressively
            double speed_curve_pow = std::max(1., 0.5 + 33 * dt);
            
            // Calculate target speed for given CTE
            double target_speed = std::max(0.65, std::pow(std::max(0., 3.5 - std::abs(cte)) / 3.5, speed_curve_pow)) * 100.;
            // Speed error is difference between target and actual speed
            double speed_error = target_speed - speed;
            // Update throttle error
            pid_throttle.UpdateError(speed_error);
            // Calculate throttle
            throttle = twiddle_enabled ? throttle : pid_throttle.TotalError();
            std::cout << "Throttle:" << throttle << " Speed error:" << speed_error << " Speed target: " << target_speed << " CTE: " << cte << " dt:" << dt << std::endl;

            json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = throttle;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            
            // Increase twiddle step counter
            twiddle_count++;
            // Calculate twiddle error
            twiddle_error += std::pow(cte, 8);
            
            // If we already know this error is too high
            if (twiddle_error > best_error)
            {
              // Stop this twiddle step
              twiddle_count = twiddle_one_step_count;
              track_completed = false;
            }
            
            // If cte is more then 3.5, assume that track was not completed
            if (std::abs(cte) > 3.5)
            {
              track_completed = false;
            }
          }
          // This simulation step is over. Start new one.
          else
          {
            // If new error is better then best error
            if (twiddle_error < best_error)
            {
              // Increase delta for current variable
              dp_vars[cur_variable] *= 1.1;
              // Make new error the best error
              best_error = twiddle_error;
              // Move on to next variable
              cur_variable = (cur_variable + 1) % 3;
              // Try addition as a next step
              cur_direction = 1;
              // If track was completed during this step, increase speed and reset error
              if (track_completed)
              {
                throttle *= 1.1;
                best_error = INFINITY;
              }
            }
            // New error is worse then previous
            else
            {
              // Restore initial values for p
              p_vars[cur_variable] -= cur_direction * dp_vars[cur_variable];
              // If we have been adding before
              if (cur_direction == 1) {
                // Try substracting now
                cur_direction = -1;
              }
              // If substraction failed
              else
              {
                // Decrease rate of advancement
                dp_vars[cur_variable] *= 0.9;
                // Move on to next variable
                cur_variable = (cur_variable + 1) % 3;
                // Try addition first
                cur_direction = 1;
              }
            }
        
            // Update variables with new settings
            p_vars[cur_variable] += cur_direction * dp_vars[cur_variable];
            
            // Reset the simulator for new twiddle step
            std::string msg("42[\"reset\", {}]");
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            
            // Reset twiddle step count
            twiddle_count = 0;
            
            // Reset track completed flag
            track_completed = true;
            
            // Update PID with new variable selection
            pid.Init(p_vars[0], p_vars[1], p_vars[2]);
            
            // Reset errors for the pid
            pid.i_error = 0;
            pid.p_error = 0;
            pid.d_error = 0;
            
            // Reset twiddle error
            twiddle_error = 0;
            
            // Output new PID params
            std::cout << "Kp:" << p_vars[0] << " Ki:" << p_vars[1] << " Kd:" << p_vars[2] << " Err:" << best_error << std::endl;
          }
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

  h.onConnection([&](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
    // Record current time
    last_time = std::chrono::high_resolution_clock::now();
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
