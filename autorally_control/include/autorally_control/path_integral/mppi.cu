template <class CONTROLLER_T, class DYNAMICS_T, class COSTS_T>
inline MPPI<CONTROLLER_T, DYNAMICS_T, COSTS_T>::~MPPI()
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

template <class CONTROLLER_T, class DYNAMICS_T, class COSTS_T>
void MPPI<CONTROLLER_T, DYNAMICS_T, COSTS_T>::onInit()
{
    //Ros node initialization
    global_node = getNodeHandle();
    mppi_node = getPrivateNodeHandle();
    is_alive_.store(true);
    bool is_nodelet = true;

    //Load setup parameters
    loadParams(&params, mppi_node);

    //Define the mppi costs
    costs = new COSTS_T(mppi_node);

    //Define the internal dynamics model for mppi
    control_constraints[0] = make_float2(-.99, .99);
    control_constraints[1] = make_float2(-.99, params.max_throttle);
    model = new DYNAMICS_T(1.0/params.hz, control_constraints);
    model->loadParams(params.model_path); //Load the model parameters from the launch file specified path

    int optimization_stride = getRosParam<int>("optimization_stride", mppi_node);

    //Define the controller
    init_u[0] = (float)params.init_steering;
    init_u[1] = (float)params.init_throttle;
    exploration_std[0] = (float)params.steering_std;
    exploration_std[1] = (float)params.throttle_std;
    mppi = new CONTROLLER_T(model, costs, params.num_timesteps, params.hz, params.gamma, exploration_std, 
                          init_u, params.num_iters, optimization_stride);

    robot = new AutorallyPlant(global_node, mppi_node, params.debug_mode, params.hz, is_nodelet);
    callback_f = boost::bind(&AutorallyPlant::dynRcfgCall, robot, _1, _2);
    server.setCallback(callback_f);
    optimizer = boost::thread(&runControlLoop<CONTROLLER_T>, mppi, robot, &params, &mppi_node, &is_alive_);
}
