# Model Predictive Control

This project controls the steering of a vehicle in a driving simulator
using a model to predict the vehicle behavior and optimize the

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

* ["ptsx"] => 
* ["ptsy"] => 
* ["x"] => 
* ["y"] => 
* ["psi"] => 
* ["speed"] => 

OUTPUT: values provided by the c++ program to the simulator

* ["steering_angle"] <= the steering command as computed by the MPC controller
* ["throttle"] <= the throttle as computed by the MPC controller
* ["mpc_x"] <= 
* ["mpc_y"] <= 
* ["next_x"] <= 
* ["next_y"] <= 

## Discussion

### Handling Latency

### Some Observations


### Results

Here is a screenshot of the simulation in progress showing the MPC algorithm in action:

![test result][image01]
