/*
 * mpc_ros
 * Copyright (c) 2021, Geonhee Lee
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

#ifndef BICYCLE_KINEMATIC_MODEL_H_
#define BICYCLE_KINEMATIC_MODEL_H_

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <Eigen/Core>
#include <geometry_msgs/Twist.h>

namespace mpc_ros {

class BicycleKinematicModel
{
 public:
    //! Default constructor
    BicycleKinematicModel(){}

    //! Constructs model with given wheelbase
    BicycleKinematicModel(double lr, double lf) : _lr(lr), _lf(lf) {}


    // implements interface method
    void getBicycleStates(const Eigen::Ref<const Eigen::VectorXd>& x, const Eigen::Ref<const Eigen::VectorXd>& u, Eigen::Ref<Eigen::VectorXd> f) 
    {
        double beta = std::atan(_lr / (_lf + _lr) * std::tan(u[1]));
        f[0] = u[0] * std::cos(x[2] + beta);
        f[1] = u[0] * std::sin(x[2] + beta);
        f[2] = u[0] * std::sin(beta) / _lr;
    }

    // implements interface method
    bool getTwistFromControl(const Eigen::Ref<const Eigen::VectorXd>& u, geometry_msgs::Twist& twist)
    {
        twist.linear.x = u[0];
        twist.linear.y = twist.linear.z = 0;

        twist.angular.z = u[1];  // warning, this is the angle and not the angular vel
        twist.angular.x = twist.angular.y = 0;

        return true;
    }

 protected:
    double _lr = 1.0;
    double _lf = 1.0;
};

}

#endif  // BICYCLE_KINEMATIC_MODEL_H_