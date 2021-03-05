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

#ifndef MPC_LOCAL_PLANNER_ROS_H
#define MPC_LOCAL_PLANNER_ROS_H

#include <vector>
#include <map>
#include <Eigen/Core>
#include <models/unicycle_kinematic_model.h>
#include <models/bicycle_kinematic_model.h>
#include <models/holonomic_kinematic_model.h>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

using namespace std;

namespace mpc_ros {
class MPC
{
    public:
        MPC();
    
        // Solve the model given an initial state and polynomial coefficients.
        // Return the first actuatotions.
        vector<double> unicycleModelSolve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
        vector<double> bicycleModelSolve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
        vector<double> holonomicModelSolve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
        vector<double> mpc_x;
        vector<double> mpc_y;
        vector<double> mpc_theta;

        void LoadParams(const std::map<string, double> &params);
    
    private:
        // Parameters for mpc solver
        double _max_angvel, _max_throttle, _bound_value;
        int _mpc_steps, _x_start, _y_start, _theta_start, _v_start, _cte_start, _etheta_start, _angvel_start, _a_start;
        int _vx_start, _vy_start, _cte_h_start, _etheta_h_start, _angvel_h_start, _ax_start, _ay_start;

        std::map<string, double> _params;
        std::string model_type;

        unsigned int dis_cnt;
        
};

}

#endif /* MPC_H */
