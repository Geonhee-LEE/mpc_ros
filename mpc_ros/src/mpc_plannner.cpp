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

#include "mpc_plannner.h"
//#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <Eigen/Core>

// The program use fragments of code from
// https://github.com/udacity/CarND-MPC-Quizzes

using CppAD::AD;

namespace mpc_ros {
// ====================================
// MPC class definition implementation.
// ====================================
MPC::MPC() 
{
    // Set default value    
    _mpc_steps = 20;
    _max_angvel = 3.0; // Maximal angvel radian (~30 deg)
    _max_throttle = 1.0; // Maximal throttle accel
    _bound_value  = 1.0e3; // Bound value for other variables

//----------------------------------------------------
    _x_start     = 0;
    _y_start     = _x_start + _mpc_steps;
    _theta_start   = _y_start + _mpc_steps;

//--------DDMR & Ackermann-------------------------

    _v_start     = _theta_start + _mpc_steps;    
    _cte_start   = _v_start + _mpc_steps;

    _etheta_start  = _cte_start + _mpc_steps;
    _angvel_start = _etheta_start + _mpc_steps;
    _a_start     = _angvel_start + _mpc_steps - 1;

//--------Holonomic--------------------------------

    _vx_start = _theta_start + _mpc_steps;
    _vy_start = _vx_start + _mpc_steps;
    _cte_h_start   = _vy_start + _mpc_steps;
    _etheta_h_start  = _cte_h_start + _mpc_steps;

    _angvel_h_start = _etheta_h_start + _mpc_steps;
    _ax_start     = _angvel_h_start + _mpc_steps - 1;
    _ay_start     = _ax_start + _mpc_steps - 1;


}

void MPC::LoadParams(const std::map<string, double> &params)
{
    _params = params;
    //Init parameters for MPC object
    _mpc_steps = _params.find("STEPS") != _params.end() ? _params.at("STEPS") : _mpc_steps;
    _max_angvel = _params.find("ANGVEL") != _params.end() ? _params.at("ANGVEL") : _max_angvel;
    _max_throttle = _params.find("MAXTHR") != _params.end() ? _params.at("MAXTHR") : _max_throttle;
    _bound_value  = _params.find("BOUND") != _params.end()  ? _params.at("BOUND") : _bound_value;
    

    //----------------------------------------------------
    _x_start     = 0;
    _y_start     = _x_start + _mpc_steps;
    _theta_start   = _y_start + _mpc_steps;

    //--------DDMR & Ackermann-------------------------

    _v_start     = _theta_start + _mpc_steps;    
    _cte_start   = _v_start + _mpc_steps;

    _etheta_start  = _cte_start + _mpc_steps;
    _angvel_start = _etheta_start + _mpc_steps;
    _a_start     = _angvel_start + _mpc_steps - 1;

    //--------Holonomic--------------------------------
    _vx_start = _theta_start + _mpc_steps;
    _vy_start = _vx_start + _mpc_steps;
    _cte_h_start   = _vy_start + _mpc_steps;
    _etheta_h_start  = _cte_h_start + _mpc_steps;
    _angvel_h_start = _etheta_h_start + _mpc_steps;
    _ax_start     = _angvel_h_start + _mpc_steps - 1;
    _ay_start     = _ax_start + _mpc_steps - 1;


    cout << "\n!! MPC Obj parameters updated !! " << endl; 
}

vector<double> MPC::unicycleModelSolve(Eigen::VectorXd state, Eigen::VectorXd coeffs) 
{
    bool ok = true;
    size_t i;
    typedef CPPAD_TESTVECTOR(double) Dvector;
    const double x = state[0];
    const double y = state[1];
    const double theta = state[2];
    const double v = state[3];
    const double cte = state[4];
    const double etheta = state[5];

    // Set the number of model variables (includes both states and inputs).
    // For example: If the state is a 4 element vector, the actuators is a 2
    // element vector and there are 10 timesteps. The number of variables is:
    // 4 * 10 + 2 * 9
    size_t n_vars = _mpc_steps * 6 + (_mpc_steps - 1) * 2;
    
    // Set the number of constraints
    size_t n_constraints = _mpc_steps * 6;

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; i++) 
    {
        vars[i] = 0;
    }

    // Set the initial variable values
    vars[_x_start] = x;
    vars[_y_start] = y;
    vars[_theta_start] = theta;
    vars[_v_start] = v;
    vars[_cte_start] = cte;
    vars[_etheta_start] = etheta;

    // Set lower and upper limits for variables.
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
    
    // Set all non-actuators upper and lowerlimits
    // to the max negative and positive values.
    for (int i = 0; i < _angvel_start; i++) 
    {
        vars_lowerbound[i] = -_bound_value;
        vars_upperbound[i] = _bound_value;
    }
    // The upper and lower limits of angvel are set to -25 and 25
    // degrees (values in radians).
    for (int i = _angvel_start; i < _a_start; i++) 
    {
        vars_lowerbound[i] = -_max_angvel;
        vars_upperbound[i] = _max_angvel;
    }
    // Acceleration/decceleration upper and lower limits
    for (int i = _a_start; i < n_vars; i++)  
    {
        vars_lowerbound[i] = -_max_throttle;
        vars_upperbound[i] = _max_throttle;
    }


    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (int i = 0; i < n_constraints; i++)
    {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }
    constraints_lowerbound[_x_start] = x;
    constraints_lowerbound[_y_start] = y;
    constraints_lowerbound[_theta_start] = theta;
    constraints_lowerbound[_v_start] = v;
    constraints_lowerbound[_cte_start] = cte;
    constraints_lowerbound[_etheta_start] = etheta;
    constraints_upperbound[_x_start] = x;
    constraints_upperbound[_y_start] = y;
    constraints_upperbound[_theta_start] = theta;
    constraints_upperbound[_v_start] = v;
    constraints_upperbound[_cte_start] = cte;
    constraints_upperbound[_etheta_start] = etheta;

    // object that computes objective and constraints
    UnicycleKinematicModel model(coeffs);
    model.loadParams(_params);

    // options for IPOPT solver
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          0.5\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, UnicycleKinematicModel>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, model, solution);

    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    // Cost
    auto cost = solution.obj_value;
    std::cout << "------------ Total Cost(solution): " << cost << "------------" << std::endl;
    cout << "max_angvel:" << _max_angvel <<endl;
    cout << "max_throttle:" << _max_throttle <<endl;
 
    cout << "-----------------------------------------------" <<endl;

    this->mpc_x = {};
    this->mpc_y = {};
    this->mpc_theta = {};
    for (int i = 0; i < _mpc_steps; i++) 
    {
        this->mpc_x.push_back(solution.x[_x_start + i]);
        this->mpc_y.push_back(solution.x[_y_start + i]);
        this->mpc_theta.push_back(solution.x[_theta_start + i]);
    }
    
    vector<double> result;
    result.push_back(solution.x[_angvel_start]);
    result.push_back(solution.x[_a_start]);
    return result;
}

vector<double> MPC::bicycleModelSolve(Eigen::VectorXd state, Eigen::VectorXd coeffs) 
{
    bool ok = true;
    size_t i;
    typedef CPPAD_TESTVECTOR(double) Dvector;
    const double x = state[0];
    const double y = state[1];
    const double theta = state[2];
    const double v = state[3];
    
    const double cte = state[4];
    const double etheta = state[5];

    // Set the number of model variables (includes both states and inputs).
    // For example: If the state is a 4 element vector, the actuators is a 2
    // element vector and there are 10 timesteps. The number of variables is:
    // 4 * 10 + 2 * 9
    size_t n_vars = _mpc_steps * 6 + (_mpc_steps - 1) * 2;
    
    // Set the number of constraints
    size_t n_constraints = _mpc_steps * 6;

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; i++) 
    {
        vars[i] = 0;
    }

    // Set the initial variable values
    vars[_x_start] = x;
    vars[_y_start] = y;
    vars[_theta_start] = theta;
    vars[_v_start] = v;
    vars[_cte_start] = cte;
    vars[_etheta_start] = etheta;

    // Set lower and upper limits for variables.
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
    
    // Set all non-actuators upper and lowerlimits
    // to the max negative and positive values.
    for (int i = 0; i < _angvel_start; i++) 
    {
        vars_lowerbound[i] = -_bound_value;
        vars_upperbound[i] = _bound_value;
    }
    // The upper and lower limits of angvel are set to -25 and 25
    // degrees (values in radians).
    for (int i = _angvel_start; i < _a_start; i++) 
    {
        vars_lowerbound[i] = -_max_angvel;
        vars_upperbound[i] = _max_angvel;
    }
    // Acceleration/decceleration upper and lower limits
    for (int i = _a_start; i < n_vars; i++)  
    {
        vars_lowerbound[i] = -_max_throttle;
        vars_upperbound[i] = _max_throttle;
    }


    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (int i = 0; i < n_constraints; i++)
    {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }
    constraints_lowerbound[_x_start] = x;
    constraints_lowerbound[_y_start] = y;
    constraints_lowerbound[_theta_start] = theta;
    constraints_lowerbound[_v_start] = v;
    constraints_lowerbound[_cte_start] = cte;
    constraints_lowerbound[_etheta_start] = etheta;
    constraints_upperbound[_x_start] = x;
    constraints_upperbound[_y_start] = y;
    constraints_upperbound[_theta_start] = theta;
    constraints_upperbound[_v_start] = v;
    constraints_upperbound[_cte_start] = cte;
    constraints_upperbound[_etheta_start] = etheta;

    // object that computes objective and constraints
    BicycleKinematicModel model(coeffs);
    model.loadParams(_params);


    // options for IPOPT solver
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          0.5\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, BicycleKinematicModel>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, model, solution);

    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    // Cost
    auto cost = solution.obj_value;
    std::cout << "------------ Total Cost(solution): " << cost << "------------" << std::endl;
    cout << "max_angvel:" << _max_angvel <<endl;
    cout << "max_throttle:" << _max_throttle <<endl;
 
    cout << "-----------------------------------------------" <<endl;

    this->mpc_x = {};
    this->mpc_y = {};
    this->mpc_theta = {};
    for (int i = 0; i < _mpc_steps; i++) 
    {
        this->mpc_x.push_back(solution.x[_x_start + i]);
        this->mpc_y.push_back(solution.x[_y_start + i]);
        this->mpc_theta.push_back(solution.x[_theta_start + i]);
    }
    
    vector<double> result;
    result.push_back(solution.x[_angvel_start]);
    result.push_back(solution.x[_a_start]);
    return result;
}

vector<double> MPC::holonomicModelSolve(Eigen::VectorXd state, Eigen::VectorXd coeffs) 
{
    cout << "Start holonomic solve" << endl;

    bool ok = true;
    size_t i;
    typedef CPPAD_TESTVECTOR(double) Dvector;
    const double x = state[0];
    const double y = state[1];
    const double theta = state[2];
    const double v_x = state[3];
    const double v_y = state[4];
    const double cte = state[5];
    const double etheta = state[6];

    // Set the number of model variables (includes both states and inputs).
    // For example: If the state is a 4 element vector, the actuators is a 2
    // element vector and there are 10 timesteps. The number of variables is:
    // 4 * 10 + 2 * 9
    size_t n_vars = _mpc_steps * 7 + (_mpc_steps - 1) * 3;
    
    // Set the number of constraints
    size_t n_constraints = _mpc_steps * 7;

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; i++) 
    {
        vars[i] = 0;
    }

    // Set the initial variable values
    vars[_x_start] = x;
    vars[_y_start] = y;
    vars[_theta_start] = theta;
    vars[_vx_start] = v_x;
    vars[_vy_start] = v_y;
    vars[_cte_h_start] = cte;
    vars[_etheta_h_start] = etheta;

    // Set lower and upper limits for variables.
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
    
    // Set all non-actuators upper and lowerlimits
    // to the max negative and positive values.
    for (int i = 0; i < _angvel_h_start; i++) 
    {
        vars_lowerbound[i] = -_bound_value;
        vars_upperbound[i] = _bound_value;
    }
    // The upper and lower limits of angvel are set to -25 and 25
    // degrees (values in radians).
    for (int i = _angvel_h_start; i < _ax_start; i++) 
    {
        vars_lowerbound[i] = -_max_angvel;
        vars_upperbound[i] = _max_angvel;
    }
    // Acceleration/decceleration upper and lower limits
    
        for (int i = _ax_start; i < _ay_start; i++)  
    {
        vars_lowerbound[i] = -_max_throttle;
        vars_upperbound[i] = _max_throttle;
    }
    
    for (int i = _ay_start; i < n_vars; i++)  
    {
        vars_lowerbound[i] = -_max_throttle;
        vars_upperbound[i] = _max_throttle;
    }


    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (int i = 0; i < n_constraints; i++)
    {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }
    constraints_lowerbound[_x_start] = x;
    constraints_lowerbound[_y_start] = y;
    constraints_lowerbound[_theta_start] = theta;
    constraints_lowerbound[_vx_start] = v_x;
    constraints_lowerbound[_vy_start] = v_y;

    constraints_lowerbound[_cte_h_start] = cte;
    constraints_lowerbound[_etheta_h_start] = etheta;
    
    constraints_upperbound[_x_start] = x;
    constraints_upperbound[_y_start] = y;
    constraints_upperbound[_theta_start] = theta;
    constraints_upperbound[_vx_start] = v_x;
    constraints_upperbound[_vy_start] = v_y;
    
    constraints_upperbound[_cte_h_start] = cte;
    constraints_upperbound[_etheta_h_start] = etheta;

    cout << "constraint set" << endl;


    // object that computes objective and constraints
    HolonomicKinematicModel model(coeffs);

    cout << "holonomic model" << endl;

    model.loadParams(_params);

    cout << "holonomic model loadParam" << endl;


    // options for IPOPT solver
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          0.5\n";


    cout << "set option" << endl;

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, HolonomicKinematicModel>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, model, solution);

    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    cout << "get solve" << endl;

    // Cost
    auto cost = solution.obj_value;
    std::cout << "------------ Total Cost(solution): " << cost << "------------" << std::endl;
    cout << "max_angvel:" << _max_angvel <<endl;
    cout << "max_throttle:" << _max_throttle <<endl;
 
    cout << "-----------------------------------------------" <<endl;

    this->mpc_x = {};
    this->mpc_y = {};
    this->mpc_theta = {};
    for (int i = 0; i < _mpc_steps; i++) 
    {
        this->mpc_x.push_back(solution.x[_x_start + i]);
        this->mpc_y.push_back(solution.x[_y_start + i]);
        this->mpc_theta.push_back(solution.x[_theta_start + i]);
    }
    
    vector<double> result;
    result.push_back(solution.x[_angvel_h_start]);
    result.push_back(solution.x[_ax_start]);
    result.push_back(solution.x[_ay_start]);

    return result;
}
}