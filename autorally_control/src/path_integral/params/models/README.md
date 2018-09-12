This folder contains the parameters for the AutoRally dynamics models used by MPPI. All of the models take as input 4 (roll, longitudenal velocity, lateral velocity, heading rate) state variables and 
the commanded steering and throttle, and they output the time derivative of the state variables.

## autorally_nnet_09_12_2018.npz 
This is a neural network with 2 hidden layers and tanh non-linearities, which means that the total configuration of the network is 6-32-32-4. This network is meant to be used by the class neural_net_model.cuh, which takes the 6-32-32-4 shape as a template parameter. Within neural_net_model.cuh, the model is loaded in by the function "loadParams(std::string model_path)", where model path is the path to this file. This network was trained on real world data collected at CCRF on the beta chassis. It was initially
trained to minimize one-step prediction error, and then fine-tuned using back-prop through time in order to minimize multi-step prediction error. For both phases of training stochastic gradient descent with ADAM was used.

## basis_function_09_12_2018.npz 
This model predicts the time derivative of the state as a linear combination of a set of pre-defined basis functions. The basis functions that are used are defined 
in the file car_bfs.cuh. This model is meant to be used by the class generalized_linear.cuh, which takes the basis functions defined by car_bfs.cuh as a template parameter. Within
generalized_linear.cuh, the model is loaded in by "loadParams(std::string model_path)" where model_path is the path to this file.

## gazebo_nnet_09_12_2018.npz 
This is an old neural network based on the ROS indigo 2.x version of Gazebo. It has the exact same format as autorally_nnet.npz This model should not be used except to compare with previously published results. The paper "Robust Sampling Based Model Predictive Control with Sparse Objective Information" presented at RSS 2018 used this model with Gazebo 2.x. Note that with Gazebo 7+ the physics are vastly improved and the model trained on real-world data performs better than this network.
