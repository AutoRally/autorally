#ifndef DDP_MODEL_WRAPPER_H
#define DDP_MODEL_WRAPPER_H

#include "ddp_dynamics.h"
#include <boost/typeof/typeof.hpp>
#include <boost/type_traits.hpp>
#include <type_traits>

template<typename T>
struct HasAnalyticGrad
{
    template <typename U> static char Test( typeof(&U::computeGrad) ) ;
    template <typename U> static long Test(...);
    static const bool Has = sizeof(Test<T>(0)) == sizeof(char);
};

template<typename T>
bool getGrad(T* model, typename Dynamics<float, T::STATE_DIM, T::CONTROL_DIM>::Jacobian &jac,
            Eigen::MatrixXf &x, Eigen::MatrixXf &u, std::true_type)
{
    model->computeGrad(x, u);
    for (int i = 0; i < T::STATE_DIM; i++){
        for (int j = 0; j < T::STATE_DIM + T::CONTROL_DIM; j++){
            jac(i,j) = model->jac_(i,j);
        }
    }
    return true;
}

template<typename T>
bool getGrad(T* model, typename Dynamics<float, T::STATE_DIM, T::CONTROL_DIM>::Jacobian &jac, 
            Eigen::MatrixXf &x, Eigen::MatrixXf &u, std::false_type)
{
    return false;
}

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
        bool analyticGradComputed = getGrad(model_, j_, state, control, std::integral_constant<bool, HasAnalyticGrad<DYNAMICS_T>::Has>());
        if (!analyticGradComputed){
            j_ = Dynamics<float, DYNAMICS_T::STATE_DIM, DYNAMICS_T::CONTROL_DIM>::df(x,u); 
        }
        return j_;
    }

};

#endif // DDP_MODEL_WRAPPER_H
