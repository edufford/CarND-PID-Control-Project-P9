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

/**
 * Checks if the SocketIO event has JSON data.
 * If there is data the JSON object in string format will be returned,
 * else the empty string "" will be returned.
 */
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

/**
 * Main loop to process measurements received from Udacity simulator via
 * uWebSocket messages.  After receiving cross-track error (CTE), speed, and
 * steering angle, process it using a PID controller and send resulting
 * control steering angle value back to the simulator to drive around the track.
 */
int main() {
  uWS::Hub h;

  // Steering PID control parameters
  constexpr double kMaxI = 1.0;  // Max guard for I term
  constexpr double kMaxD = 0.2;  // Max guard for D term
  constexpr double kSmoothD = 3.0;  // Smoothing factor for D term
  constexpr double kMaxErrorRate = 0.05;  // Max rate limit for PID output
  
  // Driving parameters
  struct Drive {
    long int n_loop = 0; // Loop counter for reference
    double throttle = 0.3;  // Use a constant value for simplicity
    bool use_twiddle = false;  // flag to enable PID twiddling
    int kTwiddleNStartError = 1000;  // # of loops before twiddle error starts
    int kTwiddleNReset = 5000;  // # of loops to run each twiddle parameter set
  };

  Drive drive;
  PID pid;
  
  // Tuned PID parameters
  //pid.Init(0.1, 0.000, 2.0, 1.0, 1.0, 1.0, 1.0); // PD only
  //pid.Init(0.1, 0.000, 2.0,  kMaxI, kMaxD, kSmoothD, kMaxErrorRate); // PD only with filters
  //pid.Init(0.1, 0.001, 2.0, kMaxI, kMaxD, kSmoothD, kMaxErrorRate); // PID
  pid.Init(0.084271, 0.000690, 3.000000, kMaxI, kMaxD, kSmoothD, kMaxErrorRate); // PID twiddled
  
  //pid.Init(0.08, 0.000, 5.0, 1.0, 0.4, 1.0, 1.0); // Throttle 0.9, PD
  

  // Set debug logging decimal precision
  std::cout << std::fixed;
  std::cout << std::setprecision(6);
  
  std::cout << "\nInitial gains: Kp: " << pid.Kp_ << ", Ki: " << pid.Ki_
            << ", Kd: " << pid.Kd_ << ", kMaxI: " << kMaxI << ", kMaxD: "
            << kMaxD << ", kSmoothD: " << kSmoothD << ", kMaxErrorRate: "
            << kMaxErrorRate << std::endl;
  
  /**
   * Loop on communication message with simulator
   */
  h.onMessage([&pid, &drive](uWS::WebSocket<uWS::SERVER> ws, char *data,
                             size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;

          // Increment loop counter
          drive.n_loop += 1;

          /**
           * Twiddle algorithm to auto-tune parameters
           */
          
          if (drive.use_twiddle == true) {
            
            // Accumulate error term after driving has stabilized
            if (drive.n_loop > drive.kTwiddleNStartError) {
              pid.TwiddleErrorUpdate(cte, angle);
            }
            
            // Set a crashed flag if speed drops too low such as from going
            // off track or hitting a wall
            bool hasCrashed = ((drive.n_loop > drive.kTwiddleNStartError)
                               && (speed < 10.0));
            
            // End run and start next parameter set
            if ((drive.n_loop > drive.kTwiddleNReset) || (hasCrashed)) {

              // If run ended by crash condition, force a high value for
              // twiddle error to evaluate it as a bad parameter set.
              if (hasCrashed) { pid.twiddle_error_ = 999999.; }
              
              std::cout << "Result error: " << pid.twiddle_error_ << std::endl;
              
              // Use twiddle algorithm to decide next parameter set to try
              pid.TwiddleParamUpdate();
              
              // Debug log output of twiddle result
              std::cout << "\nTry gains: " << pid.twiddle_idx_ << ", Kp: "
                        << pid.Kp_ << ", Ki: " << pid.Ki_ << ", Kd: "
                        << pid.Kd_ << std::endl;
              std::cout << "        Delta dKp: " << pid.Kdeltas_[0]
                        << ", dKi: " << pid.Kdeltas_[1] << ", dKd: "
                        << pid.Kdeltas_[2] << std::endl;
              std::cout << "        Current best error: "
                        << pid.twiddle_best_error_ << std::endl;

              // Reset simulator drive and start next run
              drive.n_loop = 0;
              pid.Reset();
              std::string reset_msg = "42[\"reset\",{}]";
              ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
            }
          }
          
          /**
           * Steering PID control
           */
          
          // Disable I term if car is not moving fast enough (standing start)
          if (speed < 10.0) { pid.i_cut_ = true; }
          else { pid.i_cut_ = false; }

          // Update PID terms with latest CTE
          pid.UpdateError(cte);
          
          // Set control steering value from PID output
          steer_value = pid.TotalError();
          
          // Output each loop's debug log if not twiddling
          if (drive.use_twiddle == false) {
            std::cout << "N: " << drive.n_loop
                      << ", Steer: " << steer_value
                      << ", CTE: " << cte
                      << ", Speed: " << speed
                      << ", P: " << pid.p_error_
                      << ", I: " << pid.i_error_
                      << ", D: " << pid.d_error_
                      << ", Throttle: " << drive.throttle << std::endl;
          }
          
          // Send control values back to simulator
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = drive.throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        } // (event == "telemetry")
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } // websocket message 42
  }); // h.onMessage()

  // We don't need this since we're not using HTTP but if it's removed the
  // program doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req,
                     char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    }
    else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    //ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  }
  else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
