[//]: # (Image References)
[video1]: ./videos/12_0.1_no_Latency.avi
[video2]: ./videos/9_0.1_100ms_Latency.avi


# A Vehicle driving using MPC controller
Udacity's Self-Driving Car Engineer Nanodegree Program, last project for second term

---

## Project Introduction

This project implements a MPC, Model Predictive controller, in C++ to maneuver a vehicle in a simulated environment on a track. The project implementation is described in following four steps:

1. The model
2. Timestep Length and Elapsed Duration (N & dt)
3. Polynomial Fitting and MPC Preprocessing
4. Model Predictive Control with Latency

## MPC Basics

In this approach, the model is optimized to reduce the error in the cost equations. In case of a vehicle, the error is minimized to drive it at a reference speed or as close to reference lane defined by waypoints.

1. The model state is defined as _[x, y, ψ, v, cte, eψ]_ and actuator as _[δ, a]_ where
* **x**: X position
* **y**: Y position
* **ψ**: heading/orientation
* **v**: velocity
* **cte**: cross-track error
* **eψ**: heading/orientation error

and

* **δ**: delta steering angle
* **a**: actuator as throttle or braking

The MPC controller's goal is to minimize cte and eψ. The cost function measures how far the model state is from the goal.

Update equations are coded as below:

```
            // Setup the rest of the model constraints:
            // x(t+1)​=x(t) + v(t)​*cos(psi(t))∗dt
            fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);

            // y(t+1)=y(t) + v(t)*sin(psi(t))∗dt
            fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);

            // psi(t+1)=psi(t) + ((v(t)/Lf)*delta(t)∗dt)
            fg[1 + psi_start + t] = psi1 - (psi0 + (v0 / Lf) * delta0 * dt);

            // v(t+1)=v(t)+a(t)∗dt
            fg[1 + v_start + t] = v1 - (v0 + a0 * dt);

            // cte(t+1)=f(x(t))−y(t)+(v(t)*sin(epsi(t))*dt)
            fg[1 + cte_start + t] = cte1 - (f0 - y0 + (v0 * CppAD::sin(epsi0) * dt));

            // epsi(t+1)=psi(t)−psides(t)+((v(t)/Lf)*delta(t)*dt)
            fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + (v0 / Lf) * delta0 * dt);
```

2. Timestep Length (_dt_) and number of timesteps (_N_) is used to compute prediction horizon for the elapsed Duration (_T_). The values are set with trial and error approach, observing how the vehicle drives. I started with N with 12 and dt as 0.1. Increasing N caused the vehicle to be unstable especially during turns and further zigzag ride as it tries to get back to the reference line (shown here in ![video](./videos/12_0.1_no_Latency.avi)). I tried reducing dt in that case, though it caused further degraded performance as it required more computation. Later when changes made to support latency, I settled on following parameters. Other than these two parameters I also changed  `max_cpu_time` to `0.3`. The values are coded as:

```
            size_t N = 9;
            double dt = 0.1;
```

Various factors used in evaluating the cost function using Ipopt optimizer are also adjusted. These factors work as a weights to influence the cost function. The values are coded as:

```
        double cte_start_factor = 2500;
        double epsi_start_factor = 5000;
        double v_start_factor = 2;
        double delta_start_factor = 40;
        double a_start_factor = 50;
        double delta_start_diff_factor = 500;
        double a_start_diff_factor = 25;

        for (size_t t = 0; t < N; t++) {
            fg[0] += cte_start_factor * CppAD::pow(vars[cte_start + t] - ref_cte, 2);
            fg[0] += epsi_start_factor * CppAD::pow(vars[epsi_start + t] - ref_epsi, 2);
            fg[0] += v_start_factor * CppAD::pow(vars[v_start + t] - ref_v, 2);
        }

        // Minimize the use of actuators.
        for (size_t t = 0; t < N - 1; t++) {
            fg[0] += delta_start_factor * CppAD::pow(vars[delta_start + t], 2);
            fg[0] += a_start_factor * CppAD::pow(vars[a_start + t], 2);
        }

        // Minimize the value gap between sequential actuations.
        for (size_t t = 0; t < N - 2; t++) {
            fg[0] += delta_start_diff_factor * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
            fg[0] += a_start_diff_factor * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
        }
```

3. Polynomial Fitting and MPC Preprocessing is required for calculating the initial cross track error (cte) and orientation error (eψ) values. However before fitting a polynomial to the input waypoints they needs to be processed as the server returns the waypoints using the map's coordinate system. It needs to be transformed to the vehicle's coordinate system. Using rotation matrix multiplication in the function below transformation of the input coordinates is performed:

```
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
```

Following code shows the call to transform and polyfit function to obtain 3rd degree polynomial coefficients.

```
            transformMapToVehicleCoordSys(ptsx, ptsy, psi, px, py, xvals, yvals);
            auto poly_coeff = polyfit(xvals, yvals, 3);
```

Then using the coefficients in polyeval function cte is computed.  

```
            double cte = polyeval(poly_coeff, 0) - 0;
```

Finally, orientation error (eψ) is computed as:

```
            double epsi = 0 - atan(3 * poly_coeff[3] * pow(0, 2) + 2 * 0 * poly_coeff[2] + poly_coeff[1]);
```

4. Model Predictive Control with Latency of 100 milli-seconds is introduced to make realistic scenario. To simulate the actuator dynamics this 100 ms delay is introduced from the time MPC computation and sent to simulator. Using the vehicle model, the initial state is computed for the time (`dt = 0.1`) equivalent of latency. Following code sets the initial state and supplied to MPC solve:

```
            double Lf = 2.67;
            double dt = 0.1;

            double delay_x = v * dt;
            double delay_y = 0;
            double delay_psi = -(v / Lf) * steer_value  * dt;
            double delay_v = v + throttle_value * dt;
            double delay_cte = cte + v * sin(epsi) * dt;
            double delay_epsi = epsi - (v / Lf) * steer_value  * dt;

            Eigen::VectorXd state(6);
            state << delay_x, delay_y, delay_psi, delay_v, delay_cte, delay_epsi;
            auto vars = mpc.Solve(state, poly_coeff);
```

Following video ![link](./videos/9_0.1_100ms_Latency.avi) shows vehicle maneuvering using MPC with 100 ms latency.

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Build with Docker-Compose
The docker-compose can run the project into a container
and exposes the port required by the simulator to run.

1. Clone this repo.
2. Build image: `docker-compose build`
3. Run Container: `docker-compose up`
4. On code changes repeat steps 2 and 3.

## Tips

1. The MPC is recommended to be tested on examples to see if implementation behaves as desired. One possible example
is the vehicle offset of a straight line (reference). If the MPC implementation is correct, it tracks the reference line after some timesteps(not too many).
2. The `lake_track_waypoints.csv` file has waypoints of the lake track. This could fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We have kept editor configuration files out of this repo to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. We omitted IDE profiles to ensure
students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. Most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio and develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
