# Model Predictive Control

This project controls the steering of a vehicle in a driving simulator
using a model to predict the vehicle behavior and optimize the control
of the steering and throttle.

[//]: # (Image References)

[image01]: ./images/model-predictive-control.png "Model Predictive Control"

## Requirements

This project requires the Udacity Term 2 Simulator which can be downloaded
[here](https://github.com/udacity/self-driving-car-sim/releases)

It also requires the [uWebSocketIO](https://github.com/uWebSockets/uWebSockets)
library.

Here are some notes about how to build and install the proper version of the
library on an Ubuntu system:

    sudo apt-get update
    sudo apt-get install git libuv1-dev libssl-dev gcc g++ cmake make
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    mkdir build
    cd build
    cmake ..
    make
    sudo make install
    cd ../..
    sudo ln -s /usr/lib64/libuWS.so /usr/lib/libuWS.so
    sudo rm -r uWebSockets

## Other Important Dependencies

* cmake >= 3.5
* gcc/g++ >= 5.4

## Build Instructions

Once the install for uWebSocketIO is complete, the main program can be built
and run by doing the following from the project top directory.

1. Clone this repository
2. mkdir build
3. cd build
4. cmake ..
5. make

## How to Run

### Start the simulator

Start the simulator and select the EKF scenario in the simulator:

    ./term2_sim_linux/term2_sim.x86_64

### Run the MPC Controller

The executable can then be started:

    ./mpc

Once this is done just press the "Start" button in the simulator and
watch the fireworks.

### How it Works

The MPC executable is using a simple protocol using uWebSocketIO to
communicate with the simulator exchanging JSON-formatted messages.

INPUT: values provided by the simulator to the c++ program

* ["ptsx"] => x coordinates of the waypoints
* ["ptsy"] => y coordinates of the waypoints
* ["x"] => x coordinate of the current vehicle position
* ["y"] => y coordinate of the current vehicle position
* ["psi"] =>  current orientation of the vehicle (radians)
* ["speed"] =>  current speed of the vehicle (mph)

OUTPUT: values provided by the c++ program to the simulator

* ["steering_angle"] <= the steering command as computed by the MPC controller
* ["throttle"] <= the throttle command as computed by the MPC controller
* ["mpc_x"] <= MPC prediction (for visualization)
* ["mpc_y"] <= MPC prediction (for visualization)
* ["next_x"] <= waypoints coordinates (for visualization)
* ["next_y"] <= waypoints coordinates (for visualization)

The steering angle and the throttle are used to control the simulated vehicle
and the other parameters are used to display the next waypoints used as input
as well as the vehicle trajectory predicted by the MPC algorithm.

## Discussion

Here are some notes about the experiments made during the development of this project.

### Handling Latency

The implementation provided here enforces a 100ms latency between the time when the
actuators command are computed by the MPC algorithm and the moment it actually takes
effect.

Handling the latency properly required predicting the vehicle state 100ms in the
future and using this as the starting point for the MPC algorithm.

During this work the algorithm was initially implemented and tested without any latency
in order to ensure the correctness of the algorithm.

Then in a second the proper state prediction had to be determined. Without this the vehicle's
trajectory was very unstable. Most of the time the vehicle constantly turn left and right
and almost systematically end-up out of the road.

Here is the code which is predicting the vehicle state 100ms in the future before starting
the MPC algorithm itself:

~~~.c++
          // Estimate the vehicle state 100ms in the future to compensate the 100ms latency.
          Eigen::VectorXd state(6);
          double latency = 0.1;
          double delta = j[1]["steering_angle"];
          double acceleration = j[1]["throttle"];
          double mph_to_ms = 0.44704;

          double estimated_x = v * mph_to_ms * latency;

          // The cross track error is calculated by evaluating the polynomial at x = 0.0.
          double estimated_cte = polyeval(coeffs, estimated_x);

          // The angle is changing over the latency period.
          double estimated_psi = - v * mph_to_ms * delta / 2.67 * latency;
          double estimated_y = v * mph_to_ms * sin(estimated_psi);

          // The orientation error is the opposite of the polynomial derivative.
          double estimated_epsi = -atan(coeffs[1] + 2 * coeffs[2] * estimated_x + 3 * coeffs[3] * estimated_x * estimated_x) - estimated_psi;

          double estimated_v = v + acceleration * latency;
          state << estimated_x, estimated_y, estimated_psi, estimated_v, estimated_cte, estimated_epsi;

~~~

### Some Observations

#### Number of timesteps and interval

After quite some experimentation the number of timesteps  has been set to 8
and the timestep period to 0.125 second.

It was observed that an event horizon at 1 second in the future was adequate for
a vehicle driving at that speed.

The intuition is that it would probably be more appropriate to define this event
horizon more in terms of distance in front of vehicle, rather than in time.

#### Target speed

The target speed has been set to about 100 kilometers/h (60 mph).

However it is clear that introducing some logic to lower the target speed when
the curvature of trajectory defined by the waypoints augments.

This was clearly demonstrated by manual experiments where different target speeds
were used.

#### Cost function

The cost function used by the MPC algorithm for the trajectory optimization also
had to undergo quite some tuning.

There is not really a unique solution to that problem, but it is quite sure that
this implementation is not entirely stable.

Here is the relevant code defining the cost function:

~~~.c++
      // The part of the cost based on the reference state.
      for (unsigned int t = 0; t < N; t++) {
        // Give a strong preference to the optimization of CTE and EPSI second
        fg[0] += 500*CppAD::pow(vars[cte_start + t], 2);
        fg[0] += 200*CppAD::pow(vars[epsi_start + t], 2);
        // Speed is the last thing we want to optimize
        fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
      }

      // Minimize the use of actuators.
      for (unsigned int t = 0; t < N - 1; t++) {
        // Use of actuators comes third.
        fg[0] += 10*CppAD::pow(vars[steering_start + t], 2);
        fg[0] += 10*CppAD::pow(vars[throttle_start + t], 2);
      }

      // Minimize the value gap between sequential actuations.
      for (unsigned int t = 0; t < N - 2; t++) {
        // Second priority is a smooth handling of the actuators.
        fg[0] += 50*CppAD::pow(vars[steering_start + t + 1] - vars[steering_start + t], 2);
        fg[0] += 10*CppAD::pow(vars[throttle_start + t + 1] - vars[throttle_start + t], 2);
      }

~~~

While this implementation is functional and passes the tests I personally would
not jump in the vehicle without a healthy dose of functional safety checks added
to ensure that this algorithm doesn't occasionally make very bad decisions.

In particular the algorithm is extremely sensitive to the slightest variations in latency
induced by high CPU load, and could easily become unstable at high speed when this
happened on the laptop used for testing.

### Results

Here is a screenshot of the simulation in progress showing the MPC algorithm in action:

![test result][image01]
