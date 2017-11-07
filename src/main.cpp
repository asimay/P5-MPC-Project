#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/LU"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"
#include <limits>

// for convenience
using json = nlohmann::json;

#define N (7)
#define X_START (0)
#define Y_START (N)

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

Eigen::VectorXd MapToOdom(double ptx, double pty, double px, double py, double psi) {
	Eigen::VectorXd pt_waypoint = Eigen::VectorXd(3);
	pt_waypoint << ptx, pty, 1;
	
	Eigen::VectorXd pt_car = Eigen::VectorXd(3);
	pt_car << 0, 0, 1;
	
	Eigen::MatrixXd rotation(3,3);
	rotation << cos(psi),  -sin(psi), px,
	            sin(psi),   cos(psi), py,
				0,	        0,        1;
	rotation = rotation.inverse();			
				
	pt_car = rotation * pt_waypoint;
	
	return pt_car;
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
          std::cout << "----j---- " << std::endl << j << std::endl;
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
		  double steer_value = j[1]["steering_angle"];
          double throttle_value = j[1]["throttle"];
		  
		  std::cout << "start ---------------" << std::endl;
		  std::cout << "ptsx.size() ---------------"  << ptsx.size() << std::endl;
		  
		  for(int i = 0; i < (int)ptsx.size(); i++) {
			  Eigen::VectorXd point = MapToOdom(ptsx[i], ptsy[i], px, py, psi);
			  ptsx[i] = point[0];
			  ptsy[i] = point[1];
			  //Eigen::Vector2f localpt = global2local(ptsx[i], ptsy[i], px, py, psi);
			  //ptsx[i] = localpt[0];
			  //ptsy[i] = localpt[1];
		  }
		  
		  Eigen::VectorXd xvals = Eigen::VectorXd::Map(ptsx.data(), ptsx.size());
		  Eigen::VectorXd yvals = Eigen::VectorXd::Map(ptsy.data(), ptsy.size());
		  
		  Eigen::VectorXd coeffs = polyfit(xvals, yvals, 3);

		  //double cte = polyeval(coeffs, x0) - y0;
		  double fx_dot = coeffs[1]; //3 * coeffs[3] * xvals[0] * xvals[0] + 2 * coeffs[2] * xvals[0] + coeffs[1];
		  double epsi = psi - std::atan(fx_dot);
		  

		  Eigen::VectorXd state(6);
		  
		  //if(mpc.is_initialized == false) {
		      Eigen::Vector2f point_vehicle;
		      point_vehicle = global2local(ptsx[0], ptsy[0], px, py, psi);
			  cout << "point_vehicle[0]----" << point_vehicle[0] << endl;
              cout << "point_vehicle[1]----" << point_vehicle[1] << endl;
		  
		      Eigen::VectorXd point_vehicle0;
		      point_vehicle0 = MapToOdom(ptsx[0], ptsy[0], px, py, psi);
			  cout << "point_vehicle0[0]----" << point_vehicle0[0] << endl;
			  cout << "point_vehicle0[1]----" << point_vehicle0[1] << endl;
				
			  state[0] = 0;//point_vehicle[0];
			  state[1] = 0;//point_vehicle[1];
			  state[2] = 0;//psi;
			  state[3] = v;
			  
			  double cte = polyeval(coeffs, state[0]) - state[1];
					  
			  state[4] = cte;
			  state[5] = epsi;		  
			  
			  //mpc.is_initialized = true;
		  //}



		  std::cout << std::endl;
		  std::cout << "px: " << state[0] << std::endl;
		  std::cout << "py: " << state[1] << std::endl;
		  std::cout << "psi: " << state[2] << std::endl;
		  std::cout << "v: " << state[3] << std::endl;
		  std::cout << "cte: " << state[4] << std::endl;
		  std::cout << "epsi: " << state[5] << std::endl;
		  std::cout << "coeffs[0]: " << coeffs[0] << std::endl;
		  std::cout << "coeffs[1]: " << coeffs[1] << std::endl;
		  std::cout << "coeffs[2]: " << coeffs[2] << std::endl;
		  std::cout << "coeffs[3]: " << coeffs[3] << std::endl;
		  std::cout << std::endl;
		  
		  vector<double> solve = mpc.Solve(state, coeffs);

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          //double steer_value = solve[0]/deg2rad(25);
          //double throttle_value = solve[1];
		  steer_value = solve[6]; //deg2rad(25);
		  throttle_value = solve[7];
		  
          std::cout << "steer_value--" << steer_value << std::endl;
          std::cout << "throttle_value--" << throttle_value << std::endl;


          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = -1.0 * steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
		  
		  //auto coeffs3 = polyfit(xvals, yvals, 1);
		  
		  for(int i = 0; i < N; i++) {
			  mpc_x_vals.push_back(solve[X_START + i]);
			  mpc_y_vals.push_back(solve[Y_START + i]);
		  }
  
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

		  next_x_vals = ptsx;
		  next_y_vals = ptsy;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

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