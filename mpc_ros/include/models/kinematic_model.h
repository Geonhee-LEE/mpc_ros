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

#ifndef KINEMATIC_MODEL_H_
#define KINEMATIC_MODEL_H_

#include <vector>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <Eigen/Core>
#include <geometry_msgs/Twist.h>

using CppAD::AD;
using namespace std;

namespace mpc_ros {

// =========================================
// FG_eval class definition implementation.
// =========================================
class KinematicModel
{
 public:
    // Fitted polynomial coefficients
    Eigen::VectorXd coeffs;

    double _dt, _ref_cte, _ref_etheta, _ref_vel; 
    double  _w_cte, _w_etheta, _w_vel, _w_angvel, _w_accel, _w_angvel_d, _w_accel_d;
    int _mpc_steps, _x_start, _y_start, _theta_start, _v_start, _cte_start, _etheta_start, _angvel_start, _a_start;
    int _vx_start, _vy_start, _ax_start, _ay_start;

    AD<double> cost_cte, cost_etheta, cost_vel;
    AD<double> cost_vel_x, cost_vel_y;
    // Constructor
    KinematicModel(){}

    // Deconstruct 
    ~KinematicModel() {}

    // Load parameters for constraints
    virtual void loadParams(const std::map<string, double> &params) const {}

    // MPC implementation (cost func & constraints)
    // fg: function that evaluates the objective and constraints using the syntax   
    typedef CPPAD_TESTVECTOR(AD<double>) ADvector; 
    void operator()(ADvector& fg, const ADvector& vars) const {}
};

}

#endif  // KINEMATIC_MODEL_H_