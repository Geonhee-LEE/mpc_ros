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

#ifndef DIFFERENTIAL_DRIVE_KINEMATIC_MODEL_H_
#define DIFFERENTIAL_DRIVE_KINEMATIC_MODEL_H_

#include <vector>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <Eigen/Core>
#include <geometry_msgs/Twist.h>

using CppAD::AD;

namespace mpc_ros {

class DifferentialDriveKinematicModel
{
 public:
    //! Default constructor
    DifferentialDriveKinematicModel(){}

    //! Constructs model with given wheelbase
    DifferentialDriveKinematicModel(double wheel_separation) : _wheel_separation(wheel_separation) {}


    void getDifferentialDriveStates(CPPAD_TESTVECTOR(AD<double>)& x, CPPAD_TESTVECTOR(AD<double>)& u, CPPAD_TESTVECTOR(AD<double>) f) 
    {
        f[0] = u[0] * CppAD::cos(x[2]);
        f[1] = u[0] * CppAD::sin(x[2]);
        f[2] = u[1];
    }

    void getDifferentialDriveTrackError(const Eigen::Ref<const Eigen::VectorXd>& x, const Eigen::Ref<const Eigen::VectorXd>& u, Eigen::Ref<Eigen::VectorXd> f) 
    {
        f[0] = u[0] * std::cos(x[2]);
        f[1] = u[0] * std::sin(x[2]);
    }

    bool getTwistFromControl(const Eigen::Ref<const Eigen::VectorXd>& u, geometry_msgs::Twist& twist)
    {
        twist.linear.x = u[0];
        twist.linear.y = twist.linear.z = 0;

        twist.angular.z = u[1];
        twist.angular.x = twist.angular.y = 0;
        return true;
    }

 protected:
    double _wheel_separation = 1.0;
};

}

#endif  // DIFFERENTIAL_DRIVE_KINEMATIC_MODEL_H_