#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px        = j[1]["x"];
          double py        = j[1]["y"];
          double psi       = j[1]["psi"];
          double v         = j[1]["speed"];
          double delta     = j[1]["steering_angle"]; 		  
		  double a         = j[1]["throttle"];
		  
          // Transform waypoints from global to car's C.S.
		  Eigen::VectorXd ptsx_car(ptsx.size());
          Eigen::VectorXd ptsy_car(ptsy.size());  
          for (unsigned int i = 0; i < ptsx.size(); i++) {
            double x = ptsx[i] - px;
            double y = ptsy[i] - py;
            ptsx_car[i] = x * cos(-psi) - y * sin(-psi);
            ptsy_car[i] = x * sin(-psi) + y * cos(-psi);
          }
		  
          // Waypoints 3rd order fit
          auto coeffs = polyfit(ptsx_car, ptsy_car, 3);

		  // Latency in milisecs
		  int latency = 100;

/*           // CTE simplified since py = 0  after transform
          double cte = polyeval(coeffs, 0);
          // PSI-error simplified since px = 0 psi=0 after transform
          double epsi = -atan(coeffs[1]);
          // State		  
		  Eigen::VectorXd state(6);
		  state << 0.0, 0.0, 0.0, v, cte, epsi; 	 */		  
		  
	      // This is the length from front to CoG that has a similar radius.
		  const double Lf = 2.67;
          // Latency delay
          const double dt = latency / 1000.0;		  
          // Latency - predict state
		  // After transform psi is 0 => cos(0)=1 and sin(0)=0
		  // simplifies p_px, p_py and p_psi predictions
          double p_px   = 0.0 + 1.0 * v * dt;
          double p_py   = 0.0 + 0.0 * v * dt; // will be 0.0
          double p_psi  = 0.0 - v * (delta * deg2rad(25)) / Lf * dt;
          double p_v    = v + a * dt;
          //double p_cte  = cte + v * sin(epsi) * dt;
		  double p_cte  = polyeval(coeffs, p_px);
          //double p_epsi = epsi + v * -delta / Lf * dt;
		  double p_epsi = atan(2*coeffs[2]*p_px + coeffs[1]);
          // State values - predicted after latency
          Eigen::VectorXd state(6);
          state << p_px, p_py, p_psi, p_v, p_cte, p_epsi;
		  
          // Solve for steer and accel actuations
          auto sol = mpc.Solve(state,coeffs);  
		  //const double Lf = 2.67;
		  double steer_value    = sol[0] / (deg2rad(25) * 1.0) * -1; // delta positive rotates CCW or turn left
		  double throttle_value = sol[1];
		  
          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
		  
          // Display the MPC predicted trajectory by a Green line
		  // points are in reference to the vehicle's coordinate system
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
		  for (unsigned int i = 2; i < sol.size(); i+=2) {
            mpc_x_vals.push_back(sol[i]);
            mpc_y_vals.push_back(sol[i+1]);
          }
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // Display the waypoints/reference Yellow line
          // points are in reference to the vehicle's coordinate system
          vector<double> next_x_vals;
          vector<double> next_y_vals;
		  for(int i = 0; i<ptsx_car.size();i++){
            next_x_vals.push_back(ptsx_car[i]);
            next_y_vals.push_back(ptsy_car[i]);
          }
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(latency));
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
