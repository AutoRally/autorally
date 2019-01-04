#define __CUDACC_VER__ __CUDACC_VER_MAJOR__ * 10000 + __CUDACC_VER_MINOR__ * 100 + __CUDACC_VER_BUILD__

#include <autorally_control/path_integral/meta_math.h>
//Including neural net model
#ifdef MPPI_NNET_USING_CONSTANT_MEM__
__device__ __constant__ float NNET_PARAMS[param_counter(6,32,32,4)];
#endif
#include <autorally_control/path_integral/neural_net_model.cuh>
#include <autorally_control/path_integral/car_bfs.cuh>
#include <autorally_control/path_integral/generalized_linear.cuh>
#include <autorally_control/path_integral/car_kinematics.cuh>
#include <autorally_control/path_integral/costs.cuh>
#include <autorally_control/path_integral/mppi_controller.cuh>
#include <autorally_control/path_integral/run_control_loop.cuh>
#include <autorally_control/path_integral/mppi.cuh>

#include <pluginlib/class_list_macros.h>

namespace autorally_control{
  //#ifdef USE_NEURAL_NETWORK_MODEL__ /*Use neural network dynamics model*/
  //const int MPPI_NUM_ROLLOUTS__ = 1920;
  //const int BLOCKSIZE_X = 8;
  //const int BLOCKSIZE_Y = 16;
  //typedef NeuralNetModel<7,2,3,6,32,32,4> DynamicsModel;
  //#elif USE_BASIS_FUNC_MODEL__ /*Use the basis function model* */
  const int MPPI_NUM_ROLLOUTS__ = 2560;
  const int BLOCKSIZE_X = 16;
  const int BLOCKSIZE_Y = 1;
  typedef GeneralizedLinear<CarBasisFuncs, 7, 2, 25, CarKinematics, 3> DynamicsModel;
  //#endif

  //Convenience typedef for the MPPI Controller.
  typedef MPPIController<DynamicsModel,MPPICosts, MPPI_NUM_ROLLOUTS__, BLOCKSIZE_X, BLOCKSIZE_Y> Controller;
  typedef MPPI<Controller, DynamicsModel, MPPICosts> MPPINodelet;
}

PLUGINLIB_DECLARE_CLASS(autorally_control, MPPINodelet, autorally_control::MPPINodelet, nodelet::Nodelet)
