#ifndef __MPPI_CUH__
#define __MPPI_CUH__

#include "run_control_loop.cuh"
#include "autorally_plant.h"
#include "param_getter.h"
#include <autorally_control/PathIntegralParamsConfig.h>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <atomic>
#include <boost/thread/thread.hpp>

namespace autorally_control
{

template<class CONTROLLER_T, class DYNAMICS_T, class COSTS_T>
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
    COSTS_T* costs;
    DYNAMICS_T* model;
    CONTROLLER_T* mppi;
    AutorallyPlant* robot;
    boost::thread optimizer;
    float2 control_constraints[2];
    float init_u[2];
    float exploration_std[2];
    dynamic_reconfigure::Server<PathIntegralParamsConfig> server;
    dynamic_reconfigure::Server<PathIntegralParamsConfig>::CallbackType callback_f;
};

#include "mppi.cu"

}


#endif /*__MPPI_CUH__*/