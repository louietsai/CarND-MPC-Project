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
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
	  std::cout << "j[1] : " <<j[1]<<std::endl;
#if 0
	  //std::cout << "ptsx : " << ptsx << std::endl;
	  //std::cout << "ptsx : " << ptsx << std::endl;
	  std::cout << "px : " << px << std::endl;
	  std::cout << "py : " << py << std::endl;
	  std::cout << "v : " << v << std::endl;
#endif
          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */

  	  //Eigen::VectorXd xvals(6);
          //Eigen::VectorXd yvals(6);
          //xvals << ptsx[0], ptsx[1], ptsx[2], ptsx[3], ptsx[4], ptsx[5];
          //yvals << ptsy[0], ptsy[1], ptsy[2], ptsy[3], ptsy[4], ptsy[5];
	  //double* ptr = &v[0];
	  //Eigen::Map<Eigen::VectorXd> xvals(&ptsx[0], 6);

          Eigen::VectorXd xvals = Eigen::VectorXd::Map(ptsx.data(), 6);
          Eigen::VectorXd yvals = Eigen::VectorXd::Map(ptsy.data(), 6);

          // STEP 2: transform to vehicle coordinates
          for (int i = 0; i < xvals.size(); i++) {
            double x = xvals[i]-px;
            double y = yvals[i]-py;
            xvals[i] = x * cos(0-psi) - y * sin(0-psi);
            yvals[i] = x * sin(0-psi) + y * cos(0-psi);

          }

          cout << endl ;
          cout << " Px : " << px << endl;
          cout << " Py : " << py << endl;
          cout << " Ptsx : " << xvals << endl;
          cout << " Ptsy : " << yvals << endl;
	  cout << " v : " << v << endl;
          cout << endl;

	  // The polynomial is fitted to a straight line so a polynomial with
          // order 1 is sufficient.
          auto coeffs = polyfit(xvals, yvals, 1);
	  // The cross track error is calculated by evaluating at polynomial at x, f(x)
	  // and subtracting y.
          //double cte = polyeval(coeffs, px) - py;
          // In vehicle coordinates the cross-track error error cte is
          // the intercept at x = 0
	  double cte = polyeval(coeffs, 0) - 0;
	  // Due to the sign starting at 0, the orientation error is -f'(x).
	  // derivative of coeffs[0] + coeffs[1] * x -> coeffs[1]
	  double epsi = psi - atan(coeffs[1]);

          // state in vehicle coordinates: x,y and orientation are always zero
	  Eigen::VectorXd state(6);
	  //state << px, py, psi, v, cte, epsi;
	  state << 0, 0, 0, v, cte, epsi;
	  Solution vars = mpc.Solve(state, coeffs);
	  //auto vars = mpc.Solve(state, coeffs);
          std::cout << "after mpc solve"<< std::endl;

          double steer_value;
          double throttle_value;
          steer_value = vars.Delta.at(2);//vars[6];
          throttle_value = vars.A.at(2);

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          // mathematically positive angles are negative in the simulator,
          // therefore we have to feed the negative steer_value.
          // WARNING: the current simulator expects angles as a
          //    fraction of the max angle, here 25 degrees, not radians!
          //    It must be in the range [-1,1].
          // 25 degrees in radians are 0.436332.
          msgJson["steering_angle"] = -1*steer_value/0.436332;
          msgJson["throttle"] = throttle_value;

          std::cout << "before mpc set"<< std::endl;
          //Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
#if 0
          std::cout << "  x1 size: " << x1.size() << std::endl;
          for (int i = 2; i < x1.size(); i++) {
            std::cout << " i:" <<i <<"  x1: " << x1[i] << std::endl;
            if(i%2 == 0){
              mpc_x.push_back(x1[i]);
            } else {
              mpc_y.push_back(x1[i]);
            }
          }
#endif
          msgJson["mpc_x"] = vars.X;//mpc_x_vals;
          msgJson["mpc_y"] = vars.Y;//mpc_y_vals;

          std::cout << "before next vals set"<< std::endl;
          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          next_x_vals.resize(xvals.size());
          next_y_vals.resize(yvals.size());

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          // Next point
          for (int i = 0; i < xvals.size(); i++) {
            next_x_vals[i] = xvals[i];
            next_y_vals[i] = yvals[i];
          }
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          std::cout << std::endl;
          std::cout << "mpc x : "<< msgJson["mpc_x"].dump()<< std::endl;
          std::cout << "mpc y : "<< msgJson["mpc_x"].dump()<< std::endl;
          std::cout << "next x : "<< msgJson["next_x"].dump()<< std::endl;
          std::cout << "next y : "<< msgJson["next_y"].dump()<< std::endl;
          std::cout << "steering_angle : "<< msgJson["steering_angle"].dump()<< std::endl;
          std::cout << "throttle : "<< msgJson["throttle"].dump()<< std::endl;
          std::cout << std::endl;

          std::cout << "after next vals set"<< std::endl;
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
          this_thread::sleep_for(chrono::milliseconds(100));
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
