#include <autorally_control/path_integral/mppi.cuh>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(autorally_control, MPPI, autorally_control::MPPI, nodelet::Nodelet)

namespace autorally_control
{

MPPI::~MPPI()
{
    is_alive_.store(false);
    optimizer.join();
    robot->shutdown();
    mppi->deallocateCudaMem();
    delete robot;
    delete mppi;
    delete costs;
    delete model;
}

void MPPI::onInit()
{
    //Ros node initialization
    global_node = getNodeHandle();
    mppi_node = getPrivateNodeHandle();
    is_alive_.store(true);
    bool is_nodelet = true;

    //Load setup parameters
    loadParams(&params, mppi_node);

    //Define the mppi costs
    costs = new MPPICosts(mppi_node);

    //Define the internal dynamics model for mppi
    control_constraints[0] = make_float2(-.99, .99);
    control_constraints[1] = make_float2(-.99, params.max_throttle);
    model = new DynamicsModel(1.0/params.hz, control_constraints);
    model->loadParams(params.model_path); //Load the model parameters from the launch file specified path

    int optimization_stride;
    mppi_node.getParam("optimization_stride", optimization_stride);

    //Define the controller
    init_u[0] = (float)params.init_steering;
    init_u[1] = (float)params.init_throttle;
    exploration_std[0] = (float)params.steering_std;
    exploration_std[1] = (float)params.throttle_std;
    mppi = new Controller(model, costs, params.num_timesteps, params.hz, params.gamma, exploration_std, 
                          init_u, params.num_iters, optimization_stride);

    robot = new AutorallyPlant(global_node, mppi_node, params.debug_mode, params.hz, is_nodelet);

    optimizer = boost::thread(&runControlLoop<Controller>, mppi, robot, &params, &mppi_node, &is_alive_);

}

} //namespace autorally_control