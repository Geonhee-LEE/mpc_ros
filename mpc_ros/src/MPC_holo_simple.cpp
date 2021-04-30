 
/*
# Copyright 2018 HyphaROS Workshop.
# Developer: HaoChih, LIN (hypha.ros@gmail.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
*/

#include "MPC_holo.h"
//#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <Eigen/Core>

// The program use fragments of code from
// https://github.com/udacity/CarND-MPC-Quizzes

using CppAD::AD;

// =========================================
// FG_eval class definition implementation.
// =========================================
class FG_eval 
{
    public:
        // Fitted polynomial coefficients
        Eigen::VectorXd coeffs;

        double _dt, _ref_cte, _ref_etheta, _ref_vel; 
        double  _w_cte, _w_etheta, _w_vel, _w_angvel, _w_accel, _w_angvel_d, _w_accel_d;
        int _mpc_steps, _x_start, _y_start, _theta_start, _vx_start, _vy_start, _cte_start, _etheta_start, _angvel_start, _ax_start, _ay_start;

        AD<double> cost_cte, cost_etheta, cost_vel_x, cost_vel_y;
        // Constructor
        FG_eval(Eigen::VectorXd coeffs) 
        { 
            this->coeffs = coeffs; 

            // Set default value    
            _dt = 0.1;  // in sec
            _ref_cte   = 0;
            _ref_etheta  = 0;
            _ref_vel   = 0.5; // m/s
            _w_cte     = 100;
            _w_etheta    = 100;
            _w_vel     = 1;
            _w_angvel   = 100;
            _w_accel   = 50;
            _w_angvel_d = 0;
            _w_accel_d = 0;

            _mpc_steps   = 40;
            _x_start     = 0;
            _y_start     = _x_start + _mpc_steps;

            _vx_start     = _y_start + _mpc_steps;
            _vy_start     = _vx_start + _mpc_steps;
            _cte_start   = _vy_start + _mpc_steps;

            _ax_start     = _cte_start + _mpc_steps;
            _ay_start     = _ax_start + _mpc_steps - 1;
        }

        // Load parameters for constraints
        void LoadParams(const std::map<string, double> &params)
        {
            _dt = params.find("DT") != params.end() ? params.at("DT") : _dt;
            _mpc_steps = params.find("STEPS") != params.end()    ? params.at("STEPS") : _mpc_steps;
            _ref_cte   = params.find("REF_CTE") != params.end()  ? params.at("REF_CTE") : _ref_cte;
            _ref_etheta  = params.find("REF_ETHETA") != params.end() ? params.at("REF_ETHETA") : _ref_etheta;
            _ref_vel   = params.find("REF_V") != params.end()    ? params.at("REF_V") : _ref_vel;
            
            _w_cte   = params.find("W_CTE") != params.end()   ? params.at("W_CTE") : _w_cte;
            _w_etheta  = params.find("W_EPSI") != params.end()  ? params.at("W_EPSI") : _w_etheta;
            _w_vel   = params.find("W_V") != params.end()     ? params.at("W_V") : _w_vel;
            _w_angvel = params.find("W_ANGVEL") != params.end() ? params.at("W_ANGVEL") : _w_angvel;
            _w_accel = params.find("W_A") != params.end()     ? params.at("W_A") : _w_accel;
            _w_angvel_d = params.find("W_DANGVEL") != params.end() ? params.at("W_DANGVEL") : _w_angvel_d;
            _w_accel_d = params.find("W_DA") != params.end()     ? params.at("W_DA") : _w_accel_d;

            _x_start     = 0;
            _y_start     = _x_start + _mpc_steps;

            _vx_start     = _y_start + _mpc_steps;
            _vy_start     = _vx_start + _mpc_steps;
            _cte_start   = _vy_start + _mpc_steps;

            _ax_start     = _cte_start + _mpc_steps;
            _ay_start     = _ax_start + _mpc_steps - 1;

            //cout << "\n!! FG_eval Obj parameters updated !! " << _mpc_steps << endl; 
        }

        // MPC implementation (cost func & constraints)
        typedef CPPAD_TESTVECTOR(AD<double>) ADvector; 
        // fg: function that evaluates the objective and constraints using the syntax       
        void operator()(ADvector& fg, const ADvector& vars) 
        {
            cout << "operatoer start"<< endl; 

            // fg[0] for cost function
            fg[0] = 0;
            cost_cte =  0;
            cost_etheta = 0;

            cost_vel_x = 0;
            cost_vel_y = 0;

            /*
            for (int i = 0; i < _mpc_steps; i++) 
            {
                cout << i << endl;
                cout << "_x_start" << vars[_x_start + i] <<endl;
                cout << "_y_start" << vars[_y_start + i] <<endl;
                cout << "_theta_start" << vars[_theta_start + i] <<endl;
                cout << "_v_start" << vars[_v_start + i] <<endl;
                cout << "_cte_start" << vars[_cte_start + i] <<endl;
                cout << "_etheta_start" << vars[_etheta_start + i] <<endl;
            }*/
            cout << "_mpc_steps : " << _mpc_steps << endl;

            for (int i = 0; i < _mpc_steps; i++) 
            {
              fg[0] += _w_cte * CppAD::pow(vars[_cte_start + i] - _ref_cte, 2); // cross deviation error

              fg[0] += _w_vel * CppAD::pow(vars[_vx_start + i] - _ref_vel, 2); // speed_x error
              fg[0] += _w_vel * CppAD::pow(vars[_vy_start + i] - _ref_vel, 2); // speed_y error

              cost_cte +=  _w_cte * CppAD::pow(vars[_cte_start + i] - _ref_cte, 2);

              cost_vel_x +=  (_w_vel * CppAD::pow(vars[_vx_start + i] - _ref_vel, 2)); 
              cost_vel_y +=  (_w_vel * CppAD::pow(vars[_vy_start + i] - _ref_vel, 2)); 
            
            }            

            // Minimize the use of actuators.
            for (int i = 0; i < _mpc_steps - 1; i++) {
  
              fg[0] += _w_accel * CppAD::pow(vars[_ax_start + i], 2);
              fg[0] += _w_accel * CppAD::pow(vars[_ay_start + i], 2); //error
            }
            cout << "cost of actuators: " << fg[0] << endl; 

            // Minimize the value gap between sequential actuations.
            for (int i = 0; i < _mpc_steps - 2; i++) {
              fg[0] += _w_accel_d * CppAD::pow(vars[_ax_start + i + 1] - vars[_ax_start + i], 2);
              fg[0] += _w_accel_d * CppAD::pow(vars[_ay_start + i + 1] - vars[_ay_start + i], 2);

            }
            cout << "cost of gap: " << fg[0] << endl; 

            // fg[x] for constraints
            // Initial constraints
            fg[1 + _x_start] = vars[_x_start];
            fg[1 + _y_start] = vars[_y_start];

            fg[1 + _vx_start] = vars[_vx_start];
            fg[1 + _vy_start] = vars[_vy_start];
            fg[1 + _cte_start] = vars[_cte_start];

            // Add system dynamic model constraint
            for (int i = 0; i < _mpc_steps - 1; i++)
            {
                // The state at time t+1 .
                AD<double> x1 = vars[_x_start + i + 1];
                AD<double> y1 = vars[_y_start + i + 1];

                AD<double> vx1 = vars[_vx_start + i + 1];
                AD<double> vy1 = vars[_vy_start + i + 1];

                AD<double> cte1 = vars[_cte_start + i + 1];

                // The state at time t.
                AD<double> x0 = vars[_x_start + i];
                AD<double> y0 = vars[_y_start + i];


                AD<double> vx0 = vars[_vx_start + i];
                AD<double> vy0 = vars[_vy_start + i];
                
                AD<double> cte0 = vars[_cte_start + i];

                // Only consider the actuation at time t.
                //AD<double> angvel0 = vars[_angvel_start + i];
                AD<double> ax0 = vars[_ax_start + i];
                AD<double> ay0 = vars[_ay_start + i];

                //AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
                AD<double> f0 = 0.0;
                for (int i = 0; i < coeffs.size(); i++) 
                {
                    f0 += coeffs[i] * CppAD::pow(x0, i);
                }

                //AD<double> trj_grad0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));
                AD<double> trj_grad0 = 0.0;
                for (int i = 1; i < coeffs.size(); i++) 
                {
                    trj_grad0 += i*coeffs[i] * CppAD::pow(x0, i-1); // f'(x0)
                }
                trj_grad0 = CppAD::atan(trj_grad0);

                // Here's `x` to get you started.
                // The idea here is to constraint this value to be 0.
                //
                // NOTE: The use of `AD<double>` and use of `CppAD`!
                // This is also CppAD can compute derivatives and pass
                // these to the solver.
                // TODO: Setup the rest of the model constraints
                fg[2 + _x_start + i] = x1 - (x0 + vx0 * _dt);
                fg[2 + _y_start + i] = y1 - (y0 + vy0 * _dt);

                fg[2 + _vx_start + i] = vx1 - (vx0 + ax0 * _dt); //ax0 * CppAD::sin(theta0) * dt + ay0 * CppAD::cos(theta0) * dt
                fg[2 + _vy_start + i] = vy1 - (vy0 + ay0 * _dt); //ax0 * CppAD::cos(theta0) * dt + ay0 * CppAD::sin(theta0) * dt

                fg[2 + _cte_start + i] = cte1 - ((f0 - y0) + (vx0  * _dt));

            }
            cout << "operatoer: " << fg.size() << endl; 

        }
};

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

    _x_start     = 0;
    _y_start     = _x_start + _mpc_steps;
    _vx_start     = _y_start + _mpc_steps;
    _vy_start     = _vx_start + _mpc_steps;
    _cte_start   = _vy_start + _mpc_steps;

    _ax_start     = _cte_start + _mpc_steps;
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
    
    _x_start     = 0;
    _y_start     = _x_start + _mpc_steps;

    _vx_start     = _y_start + _mpc_steps;
    _vx_start     = _vx_start + _mpc_steps;
    _cte_start   = _vy_start + _mpc_steps;

    _ax_start     = _cte_start + _mpc_steps;
    _ay_start     = _ax_start + _mpc_steps - 1;

    cout << "\n!! MPC Obj parameters updated !! " << endl; 
}


vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) 
{

    std::cout << "------------ Start solve ------------" << std::endl;

    
    bool ok = true;
    size_t i;
    typedef CPPAD_TESTVECTOR(double) Dvector;
    /*
    const double x = state[0];
    const double y = state[1];
    const double theta = state[2];
    const double vx = state[3];
    const double vy = state[4];

    const double cte = state[5];
    const double etheta = state[6];
    */
    const double x = state[0];
    const double y = state[1];
    const double vx = state[2];
    const double vy = state[3];

    const double cte = state[4];

    // Set the number of model variables (includes both states and inputs).
    // For example: If the state is a 4 element vector, the actuators is a 2
    // element vector and there are 10 timesteps. The number of variables is:
    // 4 * 10 + 2 * 9

    size_t n_vars = _mpc_steps * 5 + (_mpc_steps -1 ) * 2;
    
    std::cout << "------- n_vars : " << n_vars << std::endl;

    // Set the number of constraints
    size_t n_constraints = _mpc_steps * 5;

    std::cout << "------- n_constaraints : " << n_constraints << std::endl;


    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.

    
    Dvector vars(n_vars);
    std::cout << "------- n_vars : " << n_vars << std::endl;

    std::cout << "------------ Set Vars : " << vars.size() << std::endl;

    for (int i = 0; i < n_vars; i++) 
    {
        vars[i] = 0;
    }

    std::cout << "------------ Set Vars : " << vars.size() << std::endl;


    // Set the initial variable values
    vars[_x_start] = x;
    vars[_y_start] = y;

    vars[_vx_start] = vx;
    vars[_vy_start] = vy;

    vars[_cte_start] = cte;

    // Set lower and upper limits for variables.
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
    
    // Set all non-actuators upper and lowerlimits
    // to the max negative and positive values.
    for (int i = 0; i < _ax_start; i++) 
    {
        vars_lowerbound[i] = -_bound_value;
        vars_upperbound[i] = _bound_value;
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

    std::cout << "------------ Set constraints : " << constraints_lowerbound.size() << std::endl;

    //------------------------------------------------

    constraints_lowerbound[_x_start] = x;
    constraints_lowerbound[_y_start] = y;
    constraints_lowerbound[_vx_start] = vx;
    constraints_lowerbound[_vy_start] = vy;

    constraints_lowerbound[_cte_start] = cte;
    
    //------------------------------------------------

    constraints_upperbound[_x_start] = x;
    constraints_upperbound[_y_start] = y;
    constraints_upperbound[_vx_start] = vx;
    constraints_upperbound[_vy_start] = vy;

    constraints_upperbound[_cte_start] = cte;
    std::cout << "------------ Set FG_eval  ------------" << std::endl;

    // object that computes objective and constraints
    FG_eval fg_eval(coeffs);
    fg_eval.LoadParams(_params);

    std::cout << "------------ End FG_eval  ------------" << std::endl;

    std::cout << "------------ Set options  ------------" << std::endl;

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

    std::cout << "------------ set solve  ------------" << std::endl;

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

    std::cout << "------------ end solve  ------------" << std::endl;


    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    // Cost
    auto cost = solution.obj_value;
    std::cout << "------------ Total Cost(solution): " << cost << "------------" << std::endl;
    cout << "-----------------------------------------------" <<endl;

    this->mpc_x = {};
    this->mpc_y = {};
    for (int i = 0; i < _mpc_steps; i++) 
    {
        this->mpc_x.push_back(solution.x[_x_start + i]);
        this->mpc_y.push_back(solution.x[_y_start + i]);
    }
    vector<double> result;
    //result.push_back(solution.x[_angvel_start]);
    result.push_back(solution.x[_ax_start]);
    result.push_back(solution.x[_ay_start]);

    return result;
}
