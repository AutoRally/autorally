#ifndef DDP_MODEL_WRAPPER_H
#define DDP_MODEL_WRAPPER_H

#include "ddp_dynamics.h"

namespace autorally_control{

template <class DYNAMICS_T>
struct ModelWrapperDDP: public Dynamics<float, DYNAMICS_T::STATE_DIM, DYNAMICS_T::CONTROL_DIM>
{
    using Scalar = float;
    using State = typename Dynamics<Scalar, DYNAMICS_T::STATE_DIM, DYNAMICS_T::CONTROL_DIM>::State;
    using Control = typename Dynamics<Scalar, DYNAMICS_T::STATE_DIM, DYNAMICS_T::CONTROL_DIM>::Control;
    using Jacobian = typename Dynamics<Scalar, DYNAMICS_T::STATE_DIM, DYNAMICS_T::CONTROL_DIM>::Jacobian;

    Eigen::MatrixXf state;
    Eigen::MatrixXf control;

    DYNAMICS_T* model_;

    ModelWrapperDDP(DYNAMICS_T* model)
    {
        model_ = model;
    }

    State f(const Eigen::Ref<const State> &x, const Eigen::Ref<const Control> &u)
    {
        state = x;
        control = u;
        model_->computeKinematics(state);
        model_->computeDynamics(state, control);
        State dx;
        for (int i = 0; i < DYNAMICS_T::STATE_DIM; i++){
            dx(i) = model_->state_der_(i);
        }
        return dx;
    }

    Jacobian df(const Eigen::Ref<const State> &x, const Eigen::Ref<const Control> &u)
    {
        Jacobian j_;
        state = x;
        control = u;
        model_->computeGrad(state, control);
        for (int i = 0; i < DYNAMICS_T::STATE_DIM; i++){
            for (int j = 0; j < DYNAMICS_T::STATE_DIM + DYNAMICS_T::CONTROL_DIM; j++){
                j_(i,j) = model_->jac_(i,j);
            }
        }
        return j_;
    }

};

}

#endif // DDP_MODEL_WRAPPER_H
