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
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

// for convenience
using json = nlohmann::json;


#define LF  (2.67)
#define DT  (0.09)

// For converting back and forth between radians and degrees.
constexpr double pi() {
    return M_PI;
}
double deg2rad(double x) {
    return x * pi() / 180;
}
double rad2deg(double x) {
    return x * 180 / pi();
}

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
             0,	         0,         1;
    rotation = rotation.inverse();

    pt_car = rotation * pt_waypoint;

    return pt_car;
}


int main() {
    uWS::Hub h;

    // MPC is initialized here!
    MPC mpc;

    std::vector<double> vec_steer_value;
    std::vector<double> vec_cte_value;

    h.onMessage([&mpc, &vec_steer_value, &vec_cte_value](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
                    //std::cout << "----j---- " << std::endl << j << std::endl;
                    // j[1] is the data JSON object
                    vector<double> ptsx = j[1]["ptsx"];
                    vector<double> ptsy = j[1]["ptsy"];
                    double px = j[1]["x"];
                    double py = j[1]["y"];
                    double psi = j[1]["psi"];
                    double v = j[1]["speed"];
                    double steer_value = j[1]["steering_angle"];
                    double throttle_value = j[1]["throttle"];

                    //std::cout << "start ---------------" << std::endl;
                    //std::cout << "ptsx.size() ---------------"  << ptsx.size() << std::endl;

                    Eigen::VectorXd ptsx_vehicle(ptsx.size());
                    Eigen::VectorXd ptsy_vehicle(ptsy.size());

                    //transform from map coordinator to odom coordinator
                    for(unsigned int i = 0; i < ptsx.size(); i++) {
                        Eigen::VectorXd point = MapToOdom(ptsx[i], ptsy[i], px, py, psi);
                        ptsx_vehicle[i] = point[0];
                        ptsy_vehicle[i] = point[1];
                    }

                    //fit 3 order polynomial
                    auto coeffs = polyfit(ptsx_vehicle, ptsy_vehicle, 3);

                    //initial cte is based on vehicle origin position(x,y) : (0,0)
                    double cte = polyeval(coeffs, 0) - 0;

                    double epsi = 0 - std::atan(coeffs[1]);
                    //because of latency, we should add latency into state
                    //initial state [x,y,psi,v,cte,epsi] is [0,0,0,v,cte,epsi]
                    double predict_x = 0 + v * cos(0) * DT;
                    double predict_y = 0 + v * sin(0) * DT;
                    double predict_psi = 0 + v * (-steer_value) * DT / LF; //delta is counter-clock
                    double predict_v = v + throttle_value *  DT;
                    double predict_cte = cte + v * sin(epsi) * DT;     //polyeval(coeffs, predict_x);
                    double predict_epsi = epsi + v * (-steer_value) * DT / LF;


                    Eigen::VectorXd state(6);
                    state << predict_x, predict_y, predict_psi, predict_v, predict_cte, predict_epsi;

                    auto solve = mpc.Solve(state, coeffs);

                    /*
                    * TODO: Calculate steering angle and throttle using MPC.
                    *
                    * Both are in between [-1, 1].
                    *
                    */

                    steer_value = solve[0]/(deg2rad(25) * LF);
                    throttle_value = solve[1];

                    //std::cout << "steer_value--" << steer_value << std::endl;
                    //std::cout << "throttle_value--" << throttle_value << std::endl;

                    json msgJson;
                    // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
                    // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
                    msgJson["steering_angle"] = steer_value;
                    msgJson["throttle"] = throttle_value;

                    //draw steering value rad to degree.
                    vec_steer_value.push_back(rad2deg(steer_value));

                    vec_cte_value.push_back(cte);

                    //Display the MPC predicted trajectory
                    vector<double> mpc_x_vals = {state[0]};
                    vector<double> mpc_y_vals = {state[1]};

                    for(unsigned int i = 2; i < solve.size(); i += 2) {
                        mpc_x_vals.push_back(solve[i]);
                        mpc_y_vals.push_back(solve[i + 1]);
                    }

                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Green line

                    msgJson["mpc_x"] = mpc_x_vals;
                    msgJson["mpc_y"] = mpc_y_vals;

                    //Display the waypoints/reference line
                    vector<double> next_x_vals;
                    vector<double> next_y_vals;

                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Yellow line
                    double points_unit = 1.5;
                    int number_points = 60;

                    for(int i = 1; i < number_points; i++) {
                        double x = i * points_unit;
                        next_x_vals.push_back(x);
                        next_y_vals.push_back(polyeval(coeffs, x));
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
                    this_thread::sleep_for(chrono::milliseconds(100));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            } else {
                //use manual control to control the parameter tuning plot.

                if(vec_steer_value.size() > 0 ) {

                    plt::named_plot("steer_value", vec_steer_value, "r-" );
                    plt::named_plot("cte_value", vec_cte_value, "g-" );

                    plt::xlim(0, 200);
                    plt::ylim(-30, 30);
                    //plt::ylim(-2, 2);  //for cte
                    plt::xlabel("Iterater Times");
                    plt::ylabel("Values");
                    plt::legend();
                    plt::save("./two_value_plot.png");
                    //plt::save("./cte_plot.png");  //for cte
                    vec_steer_value.clear();
                    vec_cte_value.clear();

                    exit(1);
                }
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
