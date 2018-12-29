#include "json.hpp"
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"

// for convenience
using json = nlohmann::json;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::string;


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
    }
    else if (b1 != string::npos && b2 != string::npos) {
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


void transformMapToVehicleCoordSys(std::vector<double> &ptsx, std::vector<double> &ptsy, double psi, double px, double py, Eigen::VectorXd &xvals, Eigen::VectorXd &yvals)
{
    std::vector<double> temp_x, temp_y;
    for (size_t i = 0; i < ptsx.size(); i++)
    {
        Eigen::MatrixXd rot(2, 2), inpts(2, 1), xformpts(2, 1);
        rot << cos(-psi), -sin(-psi),
            sin(-psi), cos(psi);
        inpts << ptsx[i] - px, ptsy[i] - py;
        xformpts = rot * inpts;

        temp_x.push_back(xformpts(0, 0));
        temp_y.push_back(xformpts(1, 0));
    }
    xvals = Eigen::Map<Eigen::VectorXd>(temp_x.data(), temp_x.size());
    yvals = Eigen::Map<Eigen::VectorXd>(temp_y.data(), temp_y.size());
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
        std::cout << sdata << std::endl;
        if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
            string s = hasData(sdata);
            if (s != "") {
                auto j = json::parse(s);
                string event = j[0].get<string>();
                if (event == "telemetry") {
                    // j[1] is the data JSON object
                    std::vector<double> ptsx = j[1]["ptsx"];
                    std::vector<double> ptsy = j[1]["ptsy"];
                    double px = j[1]["x"];
                    double py = j[1]["y"];
                    double psi = j[1]["psi"];
                    double v = j[1]["speed"];

                    Eigen::VectorXd xvals(ptsx.size());
                    Eigen::VectorXd yvals(ptsy.size());
                    transformMapToVehicleCoordSys(ptsx, ptsy, psi, px, py, xvals, yvals);

                    // Using polyfit function generate a 3rd order polynomial equation for the 
                    // input x and y values representing waypoints
                    auto poly_coeff = polyfit(xvals, yvals, 3);

                    // calculate initial cross track error based on the coefficients obtainted
                    // first compute y value for given x which is zero in this case
                    // and then subtract f(x) which is also zero
                    // cte(t)​=y(t)​−f(x(t​))
                    double cte = polyeval(poly_coeff, 0) - 0;

                    // calculate initial orientation error value
                    // psi_dest​ can be calculated as the tangential angle of the polynomial f evaluated 
                    // at x(t), arctan(f′(x(t))). f′ is the derivative of the polynomial.
                    // epsi(t)​=psi(t)​−psi_dest​ where psi(t) is zero in this case
                    // epsi(t)​=psi(t) - arctac(3*poly_coeff[3]*pow(px ,2) + 2*px*poly_coeff[2] + poly_coeff[1])
                    // where px is zero for the initial state
                    double epsi = 0 - atan(3 * poly_coeff[3] * pow(0, 2) + 2 * 0 * poly_coeff[2] + poly_coeff[1]);
                    double steer_value = j[1]["steering_angle"];
                    double throttle_value = j[1]["throttle"];// TODO could be used for latency

                    double Lf = 2.67;
                    double dt = 0.1;

                    double delay_x = v * dt;
                    double delay_y = 0;
                    double delay_psi = -(v / Lf) * steer_value  * dt;
                    double delay_v = v + throttle_value * dt;
                    double delay_cte = cte + v * sin(epsi) * dt;
                    double delay_epsi = epsi - (v / Lf) * steer_value  * dt;

                    Eigen::VectorXd state(6);
                    //state << 0, 0, 0, v, cte, epsi;
                    state << delay_x, delay_y, delay_psi, delay_v, delay_cte, delay_epsi;

                    /*
                    * Calculate steering angle and throttle using MPC.
                    * Both are in between [-1, 1].
                    */
                    auto vars = mpc.Solve(state, poly_coeff);

                    //Display the waypoints/reference line
                    std::vector<double> next_x_vals;
                    std::vector<double> next_y_vals;
                    // set points for reference line
                    double poly_inc = 2.5;
                    int num_points = 25;
                    for (int i = 0; i < num_points; i++)
                    {
                        next_x_vals.push_back(poly_inc*i);
                        next_y_vals.push_back(polyeval(poly_coeff, poly_inc*i));
                    }

                    //Display the MPC predicted trajectory 
                    std::vector<double> mpc_x_vals;
                    std::vector<double> mpc_y_vals;
                    // set points for MPC calculated value 
                    for (size_t i = 0; i < vars.size(); i++)
                    {
                        if (i % 2 == 0)
                            mpc_x_vals.push_back(vars[i]);
                        else
                            mpc_y_vals.push_back(vars[i]);
                    }

                    json msgJson;
                    // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
                    // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
                    msgJson["steering_angle"] = vars[0] / (-deg2rad(25)* Lf);   // * Lf
                    msgJson["throttle"] = vars[1];

                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Green line

                    msgJson["mpc_x"] = mpc_x_vals;
                    msgJson["mpc_y"] = mpc_y_vals;

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
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            }
            else {
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
        }
        else {
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
    }
    else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}
