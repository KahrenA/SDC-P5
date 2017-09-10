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
#include <cppad/cppad.hpp>

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

//***********************************************
// Evaluate a polynomial.
//***********************************************
double polyeval(Eigen::VectorXd coeffs, double x) 
{
	double result = 0.0;
	for (int i = 0; i < coeffs.size(); i++) 
	{
		result += coeffs[i] * pow(x, i);
	}
	return result;
}

//***************************************************************************************
// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
//***************************************************************************************
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) 
{
	assert(xvals.size() == yvals.size());
	assert(order >= 1 && order <= xvals.size() - 1);

	Eigen::MatrixXd A(xvals.size(), order + 1);

	for (int i = 0; i < xvals.size(); i++) 
	{
		A(i, 0) = 1.0;
	}

	for (int j = 0; j < xvals.size(); j++) 
	{
		for (int i = 0; i < order; i++) 
		{
			A(j, i + 1) = A(j, i) * xvals(j);
    	}
  	}	

  	auto Q = A.householderQr();
  	auto result = Q.solve(yvals);
  	return result;
}

//****************************************************************************
int main() 
{
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) 
  {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
//    cout << sdata << endl;

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
          double orient = j[1]["psi"];
          double v_in = j[1]["speed"];

			double steer_angle_in = j[1]["steering_angle"];   
			double throttle_in = j[1]["throttle"];

	
          //
          // TODO: Calculate steering angle and throttle using MPC.
          // Both are in between [-1, 1].
          //
			Eigen::VectorXd ptsx_carcoords(ptsx.size());
      		Eigen::VectorXd ptsy_carcoords(ptsy.size());

			// transform waypoints to car coordinate
		 	for (unsigned int i=0; i < ptsx.size(); i++) 
			{
	       		ptsx_carcoords[i] =  ( (ptsx[i] - px) *  cos(orient) ) + ( (ptsy[i] - py) * sin(orient) );
		        ptsy_carcoords[i] =  ( (ptsx[i] - px) * -sin(orient) ) + ( (ptsy[i] - py) * cos(orient) );
     		}

			//=======================================================
      		// fit a polynomial for our waypoints
     		auto coeffs = polyfit(ptsx_carcoords, ptsy_carcoords, 3);
			//=======================================================
			
//			std::cout << "incoming px, py, orient, v_in, steer_in = " << px << "\t" <<
//					py << "\t" << orient << "\t" << v_in << "\t" << steer_angle_in << "\n"; 

			//-------KINEMATIC MODEL ------------------------
			// x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
			// y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
			// psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
			// v_[t+1] = v[t] + a[t] * dt
			//-----------------------------------------------
			double predicted_x    = 0 + v_in * mpc.latency;		
			double predicted_y    = 0;

			// psi in car coordinates will be 0 but need to add in latency effect
			double predicted_orient  = 0 + v_in/mpc.Lf * steer_angle_in * mpc.latency;

			std::cout << v_in << "\t" << steer_angle_in << "\t"<< throttle_in << "\t";
	
			double predicted_v    = v_in + (throttle_in * mpc.latency);
				
			double cte  = polyeval(coeffs, 0);		// f(x@t) - y@t where x=y=0 
			double epsi = -atan(coeffs[1]);			// psi@t - pse@desired = 0 - atan(f'(x@t) where x=0
          
			double predicted_cte  = cte + (v_in * sin(epsi) * mpc.latency);
//			std::cout << "cte = " << cte << " ==> " << "predict_cte = " << predicted_cte << "\n";
//			std::cout << cte << "\n";
		
			double predicted_epsi = epsi + (v_in/mpc.Lf * steer_angle_in * mpc.latency); 
//			std::cout << "epsi = " << epsi << " ==> " << "predicted_epsi = " << predicted_epsi << "\n";
          
  			Eigen::VectorXd state(6);
			state << predicted_x, predicted_y, predicted_orient, predicted_v, 
												predicted_cte, predicted_epsi;
 

			//=====================================
			auto result = mpc.Solve(state, coeffs);
			//=====================================

           json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].

          	double steer_value = -result[0]/deg2rad(25);		// - sign is for the simulator 
		  	double throttle_value = result[1]; 

			std::cout << steer_value << "\t" << throttle_value << "\n";

          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

		  //------------------------------------------------------------------------
          // Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's 
		  // coordinate system.. the points in the simulator are connected by a Green line

			for (unsigned int i = 2; i < result.size(); i++) 
			{
	            if(i%2 == 0)
				{
	              	mpc_x_vals.push_back(result[i]);
	            } 
				else 
				{
	              	mpc_y_vals.push_back(result[i]);
				}
			}
  		  msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

		  //------------------------------------------------------------------------
		  // Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

		next_x_vals.resize(ptsx_carcoords.size());
		next_y_vals.resize(ptsy_carcoords.size());


          //.. add (x,y) points to list here, points are in reference to the vehicle's 
		  // coordinate system the points in the simulator are connected by a Yellow line
			for (int i = 0; i < ptsx_carcoords.size(); i++) 
			{
	            next_x_vals[i] = ptsx_carcoords[i];
	            next_y_vals[i] = ptsy_carcoords[i];
	        }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
 //         std::cout << msg << std::endl;
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
