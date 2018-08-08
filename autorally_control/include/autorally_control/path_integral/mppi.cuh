#ifndef __MPPI__
#define __MPPI__

#define __CUDACC_VER__ __CUDACC_VER_MAJOR__ * 10000 + __CUDACC_VER_MINOR__ * 100 + __CUDACC_VER_BUILD__

#include "meta_math.h"
#include "param_getter.h"

#include "costs.cuh"
#include "generalized_linear.cuh"

//Including neural net model
#ifdef MPPI_NNET_USING_CONSTANT_MEM__
__device__ __constant__ float NNET_PARAMS[param_counter(6,32,32,4)];
#endif

#include "neural_net_model.cuh"

#include "car_bfs.cuh"
#include "mppi_controller.cuh"
#include "run_control_loop.cuh"

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <atomic>

namespace autorally_control
{

#ifdef USE_NEURAL_NETWORK_MODEL__
const int MPPI_NUM_ROLLOUTS__ = 1200;
const int BLOCKSIZE_X = 8;
const int BLOCKSIZE_Y = 16;
typedef NeuralNetModel<7,2,3,6,32,32,4> DynamicsModel;
#elif USE_BASIS_FUNC_MODEL__ /*Use the basis function model*/
const int MPPI_NUM_ROLLOUTS__ = 2560;
const int BLOCKSIZE_X = 16;
const int BLOCKSIZE_Y = 4;
typedef GeneralizedLinear<CarBasisFuncs, 7, 2, 25, CarKinematics, 3> DynamicsModel;
#endif

//Convenience typedef for the MPPI Controller.
typedef MPPIController<DynamicsModel, MPPICosts, MPPI_NUM_ROLLOUTS__, BLOCKSIZE_X, BLOCKSIZE_Y> Controller;

class MPPI : public nodelet::Nodelet
{

public:
    ~MPPI();
    virtual void onInit();
    std::atomic<bool> is_alive_;

private:
    ros::NodeHandle global_node;
    ros::NodeHandle mppi_node;
    SystemParams params;
    MPPICosts* costs;
    DynamicsModel* model;
    Controller* mppi;
    AutorallyPlant* robot;
    boost::thread optimizer;
    float2 control_constraints[2];
    float init_u[2];
    float exploration_std[2];
};

}


#endif /*__MPPI__*/