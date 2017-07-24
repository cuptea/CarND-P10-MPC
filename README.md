# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## The Model

Q: Student describes their model in detail. This includes the state, actuators and update equations.

A:The MPC model is able to provide steering angle and throttle control signal by solving a optimization problem, which is formulated as follows:


*Constant:

'''

            dt: gap between adjacent gaps
            N: number of steps
            v_max: max_velocity
            delta_max: largest feasible steering angle
            delta_min: smallest feasible steering angle
            a_max: largest throttle
            a_min: smallest throttle
            x_init: current x coordinate
            y_init: current y coordinate
            v_init: current velocity
            psi_init: current steering angle
            cte_init: current cross track error
            epsi_init: current orientation error

'''

* Variables:

'''

            x_i: x coordinate, i= 1 to N
            y_i: y coordinate, i= 1 to N
            psi_i: angle, i= 1 to N
            v_i: velocity, i= 1 to N
            cte_i: cross track error, i= 1 to N
            epsi_i: orientation error, i= 1 to N
            delta_i: steering angle, i= 1 to N-1
            a_i: throttle, i= 1 to N-1

'''

* Minimize  

''' 

            w_cte * sum_{i=1}^{n}(cte_i^2) +  w_epsi * sum_{i=1}^{n}(epsi_i^2) + w_v * sum_{i=1}^{n}((v_i - v_max)^2) +
            w_delta * sum_{i=1}^{n-1}(delta_i^2) + w_a * sum_{i=1}^{n-1}(a_i^2) +
            w_d_delta * sum_{i=1}^{n-2}((delta_(i+1) - delta_i)^2) + w_d_a * sum_{i=1}^{n-2}((a_(i+1) - a_i)^2)

'''

* Subject to

'''

            x_{i+1} = x{i} + v{i} * cos(psi{i}) * dt, i= 1 to N-1
            y_{i+1} = y{i} + v{i} * sin(psi{i}) * dt, i= 1 to N-1
            psi_{i+1} = psi{i} + v{i} / Lf * delta{i} * dt, i= 1 to N-1
            v_{i+1} = v{i} + a{i} * dt, i= 1 to N-1

            f_i = coeffs_3 * x_i^3 + coeffs_2 * x_i^2 + coeffs_1 * x_i + coeffs_0; i= 1 to N-1
            psides_i = atan(3 * coeffs_3 * x_i^2 + 2 * coeffs_2 * x_i + coeffs_1); i= 1 to N-1

            cte_{i+1} = f(x_i) - y_i + v_i * sin(epsi_i) * dt, i= 1 to N-1
            epsi_{i+1} = psi_i - psides_i + v_i * delta_i / Lf * dt, i= 1 to N-1
            
            delta_min <= delta_i <= delta_max, i= 1 to N-1
            a_min<= a_i <= a_max, i= 1 to N-1

            x_1 = x_init
            y_1 = y_init
            v_1 = v_init
            psi_1 = psi_init
            cte_1 = cte_init
            epsi_1 = epsi_init

'''

## Timestep Length and Elapsed Duration (N & dt)

Q: Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

A: As discussed in the lecture, we first figure out a reasonable time window, i.e. N*dt, and then fine tune N and dt. During the tuning, we try to make the N larger and dt smaller. In this project, we first use small N, such as 7, later use the current 20. The dt is always 0.1.

## Polynomial Fitting and MPC Preprocessing

Q: A polynomial is fitted to waypoints. If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.

A: Since the raw ptsx and ptsy are stored in global coordinate system, we have to turn them into the local coordinate system of the car. The detailed code could be found in between #120 to #126 lines of main.cpp.

## Model Predictive Control with Latency

Q: The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.

A: We use the Kinetic model to predict the status in 100 ms and use it as the current status and pass it to mpc::solve function.


## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
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
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
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
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
