# ifndef CPPAD_LOCAL_REV_JAC_SWEEP_HPP
# define CPPAD_LOCAL_REV_JAC_SWEEP_HPP

/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-17 Bradley M. Bell

CppAD is distributed under multiple licenses. This distribution is under
the terms of the
                    GNU General Public License Version 3.

A copy of this license is included in the COPYING file of this distribution.
Please visit http://www.coin-or.org/CppAD/ for information on other licenses.
-------------------------------------------------------------------------- */

namespace CppAD { namespace local { // BEGIN_CPPAD_LOCAL_NAMESPACE
/*!
\file rev_jac_sweep.hpp
Compute Reverse mode Jacobian sparsity patterns.
*/

/*!
\def CPPAD_REV_JAC_SWEEP_TRACE
This value is either zero or one.
Zero is the normal operational value.
If it is one, a trace of every rev_jac_sweep computation is printed.
*/
# define CPPAD_REV_JAC_SWEEP_TRACE 0

/*!
Given the sparsity pattern for the dependent variables,
RevJacSweep computes the sparsity pattern for all the independent variables.

\tparam Base
this operation sequence was recorded using AD<Base>.

\tparam Vector_set
is the type used for vectors of sets. It can be either
sparse_pack or sparse_list.

\param dependency
Are the derivatives with respect to left and right of the expression below
considered to be non-zero:
\code
	CondExpRel(left, right, if_true, if_false)
\endcode
This is used by the optimizer to obtain the correct dependency relations.

\param n
is the number of independent variables on the tape.

\param numvar
is the total number of variables on the tape; i.e.,
\a play->num_var_rec().
This is also the number of rows in the entire sparsity pattern \a RevJac.

\param play
The information stored in \a play
is a recording of the operations corresponding to a function
\f[
	F : {\bf R}^n \rightarrow {\bf R}^m
\f]
where \f$ n \f$ is the number of independent variables
and \f$ m \f$ is the number of dependent variables.

\param var_sparsity
For i = 0 , ... , \a numvar - 1,
(all the variables on the tape)
the forward Jacobian sparsity pattern for variable i
corresponds to the set with index i in \a var_sparsity.
\b
\b
\b Input:
For i = 0 , ... , \a numvar - 1,
the forward Jacobian sparsity pattern for variable i is an input
if i corresponds to a dependent variable.
Otherwise the sparsity patten is empty.
\n
\n
\b Output: For j = 1 , ... , \a n,
the sparsity pattern for the dependent variable with index (j-1)
is given by the set with index index j in \a var_sparsity.
*/

template <class Base, class Vector_set>
void rev_jac_sweep(
	const local::player<Base>* play,
	bool                       dependency,
	size_t                     n,
	size_t                     numvar,
	Vector_set&                var_sparsity
)
{
	OpCode           op;
	size_t         i_op;
	size_t        i_var;

	const addr_t*   arg = CPPAD_NULL;

	size_t            i, j, k;

	// length of the parameter vector (used by CppAD assert macros)
	const size_t num_par = play->num_par_rec();

	// check numvar argument
	CPPAD_ASSERT_UNKNOWN( numvar > 0 );
	CPPAD_ASSERT_UNKNOWN( play->num_var_rec()   == numvar );
	CPPAD_ASSERT_UNKNOWN( var_sparsity.n_set() == numvar );

	// upper limit (exclusive) for elements in the set
	size_t limit = var_sparsity.end();

	// vecad_sparsity contains a sparsity pattern for each VecAD object.
	// vecad_ind maps a VecAD index (beginning of the VecAD object)
	// to the index of the corresponding set in vecad_sparsity.
	size_t num_vecad_ind   = play->num_vec_ind_rec();
	size_t num_vecad_vec   = play->num_vecad_vec_rec();
	Vector_set  vecad_sparsity;
	pod_vector<size_t> vecad_ind;
	if( num_vecad_vec > 0 )
	{	size_t length;
		vecad_sparsity.resize(num_vecad_vec, limit);
		vecad_ind.extend(num_vecad_ind);
		j             = 0;
		for(i = 0; i < num_vecad_vec; i++)
		{	// length of this VecAD
			length   = play->GetVecInd(j);
			// set to proper index for this VecAD
			vecad_ind[j] = i;
			for(k = 1; k <= length; k++)
				vecad_ind[j+k] = num_vecad_vec; // invalid index
			// start of next VecAD
			j       += length + 1;
		}
		CPPAD_ASSERT_UNKNOWN( j == play->num_vec_ind_rec() );
	}

	// ----------------------------------------------------------------------
	// user's atomic op calculator
	atomic_base<Base>* user_atom = CPPAD_NULL; // user's atomic op calculator
	//
	// work space used by UserOp.
	vector<Base>       user_x;   // parameters in x as integers
	vector<size_t>     user_ix;  // variable indices for argument vector
	vector<size_t>     user_iy;  // variable indices for result vector
	//
	// information set by forward_user (initialization to avoid warnings)
	size_t user_old=0, user_m=0, user_n=0, user_i=0, user_j=0;
	// information set by forward_user (necessary initialization)
	enum_user_state user_state = end_user; // proper initialization
	// ----------------------------------------------------------------------
	//
	// pointer to the beginning of the parameter vector
	// (used by atomic functions
	const Base* parameter = CPPAD_NULL;
	if( num_par > 0 )
		parameter = play->GetPar();
	//
	// Initialize
	i_op = play->num_op_rec();
	play->get_op_info(--i_op, op, arg, i_var);
	CPPAD_ASSERT_UNKNOWN( op == EndOp );
# if CPPAD_REV_JAC_SWEEP_TRACE
	std::cout << std::endl;
	CppAD::vectorBool z_value(limit);
# endif
	bool more_operators = true;
	while(more_operators)
	{	bool flag; // temporary for use in switch cases
		//
		// next op
		play->get_op_info(--i_op, op, arg, i_var);

		// rest of information depends on the case
		switch( op )
		{
			case AbsOp:
			CPPAD_ASSERT_NARG_NRES(op, 1, 1);
			reverse_sparse_jacobian_unary_op(
				i_var, arg[0], var_sparsity
			);
			break;
			// -------------------------------------------------

			case AddvvOp:
			CPPAD_ASSERT_NARG_NRES(op, 2, 1);
			reverse_sparse_jacobian_binary_op(
				i_var, arg, var_sparsity
			);
			break;
			// -------------------------------------------------

			case AddpvOp:
			CPPAD_ASSERT_NARG_NRES(op, 2, 1);
			reverse_sparse_jacobian_unary_op(
				i_var, arg[1], var_sparsity
			);
			break;
			// -------------------------------------------------

			case AcosOp:
			// sqrt(1 - x * x), acos(x)
			CPPAD_ASSERT_NARG_NRES(op, 1, 2);
			reverse_sparse_jacobian_unary_op(
				i_var, arg[0], var_sparsity
			);
			break;
			// -------------------------------------------------

# if CPPAD_USE_CPLUSPLUS_2011
			case AcoshOp:
			// sqrt(x * x - 1), acosh(x)
			CPPAD_ASSERT_NARG_NRES(op, 1, 2);
			reverse_sparse_jacobian_unary_op(
				i_var, arg[0], var_sparsity
			);
			break;
# endif
			// -------------------------------------------------

			case AsinOp:
			// sqrt(1 - x * x), asin(x)
			CPPAD_ASSERT_NARG_NRES(op, 1, 2);
			reverse_sparse_jacobian_unary_op(
				i_var, arg[0], var_sparsity
			);
			break;
			// -------------------------------------------------

# if CPPAD_USE_CPLUSPLUS_2011
			case AsinhOp:
			// sqrt(1 + x * x), asinh(x)
			CPPAD_ASSERT_NARG_NRES(op, 1, 2);
			reverse_sparse_jacobian_unary_op(
				i_var, arg[0], var_sparsity
			);
			break;
# endif
			// -------------------------------------------------

			case AtanOp:
			// 1 + x * x, atan(x)
			CPPAD_ASSERT_NARG_NRES(op, 1, 2);
			reverse_sparse_jacobian_unary_op(
				i_var, arg[0], var_sparsity
			);
			break;
			// -------------------------------------------------

# if CPPAD_USE_CPLUSPLUS_2011
			case AtanhOp:
			// 1 - x * x, atanh(x)
			CPPAD_ASSERT_NARG_NRES(op, 1, 2);
			reverse_sparse_jacobian_unary_op(
				i_var, arg[0], var_sparsity
			);
			break;
# endif
			// -------------------------------------------------

			case BeginOp:
			CPPAD_ASSERT_NARG_NRES(op, 1, 1);
			more_operators = false;
			break;
			// -------------------------------------------------

			case CSkipOp:
			break;
			// -------------------------------------------------

			case CSumOp:
			reverse_sparse_jacobian_csum_op(
				i_var, arg, var_sparsity
			);
			break;
			// -------------------------------------------------

			case CExpOp:
			reverse_sparse_jacobian_cond_op(
				dependency, i_var, arg, num_par, var_sparsity
			);
			break;
			// ---------------------------------------------------

			case CosOp:
			// sin(x), cos(x)
			CPPAD_ASSERT_NARG_NRES(op, 1, 2);
			reverse_sparse_jacobian_unary_op(
				i_var, arg[0], var_sparsity
			);
			break;
			// ---------------------------------------------------

			case CoshOp:
			// sinh(x), cosh(x)
			CPPAD_ASSERT_NARG_NRES(op, 1, 2);
			reverse_sparse_jacobian_unary_op(
				i_var, arg[0], var_sparsity
			);
			break;
			// -------------------------------------------------

			case DisOp:
			CPPAD_ASSERT_NARG_NRES(op, 2, 1);
			// derivative is identically zero but dependency is not
			if( dependency ) reverse_sparse_jacobian_unary_op(
				i_var, arg[1], var_sparsity
			);
			break;
			// -------------------------------------------------

			case DivvvOp:
			CPPAD_ASSERT_NARG_NRES(op, 2, 1);
			reverse_sparse_jacobian_binary_op(
				i_var, arg, var_sparsity
			);
			break;
			// -------------------------------------------------

			case DivpvOp:
			CPPAD_ASSERT_NARG_NRES(op, 2, 1);
			reverse_sparse_jacobian_unary_op(
				i_var, arg[1], var_sparsity
			);
			break;
			// -------------------------------------------------

			case DivvpOp:
			CPPAD_ASSERT_NARG_NRES(op, 2, 1);
			reverse_sparse_jacobian_unary_op(
				i_var, arg[0], var_sparsity
			);
			break;
			// -------------------------------------------------

			case ErfOp:
			// arg[1] is always the parameter 0
			// arg[0] is always the parameter 2 / sqrt(pi)
			CPPAD_ASSERT_NARG_NRES(op, 3, 5);
			reverse_sparse_jacobian_unary_op(
				i_var, arg[0], var_sparsity
			);
			break;
			// -------------------------------------------------

			case ExpOp:
			CPPAD_ASSERT_NARG_NRES(op, 1, 1);
			reverse_sparse_jacobian_unary_op(
				i_var, arg[0], var_sparsity
			);
			break;
			// -------------------------------------------------

# if CPPAD_USE_CPLUSPLUS_2011
			case Expm1Op:
			CPPAD_ASSERT_NARG_NRES(op, 1, 1);
			reverse_sparse_jacobian_unary_op(
				i_var, arg[0], var_sparsity
			);
			break;
# endif
			// -------------------------------------------------

			case InvOp:
			CPPAD_ASSERT_NARG_NRES(op, 0, 1);
			break;
			// -------------------------------------------------

			case LdpOp:
			reverse_sparse_jacobian_load_op(
				dependency,
				op,
				i_var,
				arg,
				num_vecad_ind,
				vecad_ind.data(),
				var_sparsity,
				vecad_sparsity
			);
			break;
			// -------------------------------------------------

			case LdvOp:
			reverse_sparse_jacobian_load_op(
				dependency,
				op,
				i_var,
				arg,
				num_vecad_ind,
				vecad_ind.data(),
				var_sparsity,
				vecad_sparsity
			);
			break;
			// -------------------------------------------------

			case EqpvOp:
			case EqvvOp:
			case LtpvOp:
			case LtvpOp:
			case LtvvOp:
			case LepvOp:
			case LevpOp:
			case LevvOp:
			case NepvOp:
			case NevvOp:
			CPPAD_ASSERT_NARG_NRES(op, 2, 0);
			break;
			// -------------------------------------------------

			case LogOp:
			CPPAD_ASSERT_NARG_NRES(op, 1, 1);
			reverse_sparse_jacobian_unary_op(
				i_var, arg[0], var_sparsity
			);
			break;
			// -------------------------------------------------

# if CPPAD_USE_CPLUSPLUS_2011
			case Log1pOp:
			CPPAD_ASSERT_NARG_NRES(op, 1, 1);
			reverse_sparse_jacobian_unary_op(
				i_var, arg[0], var_sparsity
			);
			break;
# endif
			// -------------------------------------------------

			case MulpvOp:
			CPPAD_ASSERT_NARG_NRES(op, 2, 1);
			reverse_sparse_jacobian_unary_op(
				i_var, arg[1], var_sparsity
			);
			break;
			// -------------------------------------------------

			case MulvvOp:
			CPPAD_ASSERT_NARG_NRES(op, 2, 1);
			reverse_sparse_jacobian_binary_op(
				i_var, arg, var_sparsity
			);
			break;
			// -------------------------------------------------

			case ParOp:
			CPPAD_ASSERT_NARG_NRES(op, 1, 1);

			break;
			// -------------------------------------------------

			case PowvpOp:
			reverse_sparse_jacobian_unary_op(
				i_var, arg[0], var_sparsity
			);
			break;
			// -------------------------------------------------

			case PowpvOp:
			CPPAD_ASSERT_NARG_NRES(op, 2, 3);
			reverse_sparse_jacobian_unary_op(
				i_var, arg[1], var_sparsity
			);
			break;
			// -------------------------------------------------

			case PowvvOp:
			CPPAD_ASSERT_NARG_NRES(op, 2, 3);
			reverse_sparse_jacobian_binary_op(
				i_var, arg, var_sparsity
			);
			break;
			// -------------------------------------------------

			case PriOp:
			CPPAD_ASSERT_NARG_NRES(op, 5, 0);
			break;
			// -------------------------------------------------

			case SignOp:
			CPPAD_ASSERT_NARG_NRES(op, 1, 1);
			// derivative is identically zero but dependency is not
			if( dependency ) reverse_sparse_jacobian_unary_op(
				i_var, arg[0], var_sparsity
			);
			break;
			// -------------------------------------------------

			case SinOp:
			// cos(x), sin(x)
			CPPAD_ASSERT_NARG_NRES(op, 1, 2);
			reverse_sparse_jacobian_unary_op(
				i_var, arg[0], var_sparsity
			);
			break;
			// -------------------------------------------------

			case SinhOp:
			// cosh(x), sinh(x)
			CPPAD_ASSERT_NARG_NRES(op, 1, 2);
			reverse_sparse_jacobian_unary_op(
				i_var, arg[0], var_sparsity
			);
			break;
			// -------------------------------------------------

			case SqrtOp:
			CPPAD_ASSERT_NARG_NRES(op, 1, 1);
			reverse_sparse_jacobian_unary_op(
				i_var, arg[0], var_sparsity
			);
			break;
			// -------------------------------------------------

			case StppOp:
			// does not affect sparsity or dependency when both are parameters
			CPPAD_ASSERT_NARG_NRES(op, 3, 0);
			break;
			// -------------------------------------------------

			case StpvOp:
			reverse_sparse_jacobian_store_op(
				dependency,
				op,
				arg,
				num_vecad_ind,
				vecad_ind.data(),
				var_sparsity,
				vecad_sparsity
			);
			break;
			// -------------------------------------------------

			case StvpOp:
			CPPAD_ASSERT_NARG_NRES(op, 3, 0);
			// storing a parameter only affects dependency
			reverse_sparse_jacobian_store_op(
				dependency,
				op,
				arg,
				num_vecad_ind,
				vecad_ind.data(),
				var_sparsity,
				vecad_sparsity
			);
			break;
			// -------------------------------------------------

			case StvvOp:
			reverse_sparse_jacobian_store_op(
				dependency,
				op,
				arg,
				num_vecad_ind,
				vecad_ind.data(),
				var_sparsity,
				vecad_sparsity
			);
			break;
			// -------------------------------------------------

			case SubvvOp:
			CPPAD_ASSERT_NARG_NRES(op, 2, 1);
			reverse_sparse_jacobian_binary_op(
				i_var, arg, var_sparsity
			);
			break;
			// -------------------------------------------------

			case SubpvOp:
			CPPAD_ASSERT_NARG_NRES(op, 2, 1);
			reverse_sparse_jacobian_unary_op(
				i_var, arg[1], var_sparsity
			);
			break;
			// -------------------------------------------------

			case SubvpOp:
			CPPAD_ASSERT_NARG_NRES(op, 2, 1);
			reverse_sparse_jacobian_unary_op(
				i_var, arg[0], var_sparsity
			);
			break;
			// -------------------------------------------------

			case TanOp:
			// tan(x)^2, tan(x)
			CPPAD_ASSERT_NARG_NRES(op, 1, 2);
			reverse_sparse_jacobian_unary_op(
				i_var, arg[0], var_sparsity
			);
			break;
			// -------------------------------------------------

			case TanhOp:
			// tanh(x)^2, tanh(x)
			CPPAD_ASSERT_NARG_NRES(op, 1, 2);
			reverse_sparse_jacobian_unary_op(
				i_var, arg[0], var_sparsity
			);
			break;
			// -------------------------------------------------

			case UserOp:
			// start or end an atomic function call
			CPPAD_ASSERT_UNKNOWN(
				user_state == start_user || user_state == end_user
			);
			flag = user_state == end_user;
			user_atom = play->get_user_info(op, arg, user_old, user_m, user_n);
			if( flag )
			{	user_state = ret_user;
				user_i     = user_m;
				user_j     = user_n;
				//
				user_x.resize( user_n );
				user_ix.resize( user_n );
				user_iy.resize( user_m );
			}
			else
			{	user_state = end_user;
				//
				user_atom->set_old(user_old);
				user_atom->rev_sparse_jac(
					user_x, user_ix, user_iy, var_sparsity
				);
			}
			break;

			case UsrapOp:
			// parameter argument in an atomic operation sequence
			CPPAD_ASSERT_UNKNOWN( NumArg(op) == 1 );
			CPPAD_ASSERT_UNKNOWN( user_state == arg_user );
			CPPAD_ASSERT_UNKNOWN( user_i == 0 );
			CPPAD_ASSERT_UNKNOWN( user_j <= user_n );
			CPPAD_ASSERT_UNKNOWN( size_t( arg[0] ) < num_par );
			//
			--user_j;
			// argument parameter value
			user_x[user_j] = parameter[arg[0]];
			// special variable index used for parameters
			user_ix[user_j] = 0;
			//
			if( user_j == 0 )
				user_state = start_user;
			break;

			case UsravOp:
			// variable argument in an atomic operation sequence
			CPPAD_ASSERT_UNKNOWN( NumArg(op) == 1 );
			CPPAD_ASSERT_UNKNOWN( user_state == arg_user );
			CPPAD_ASSERT_UNKNOWN( user_i == 0 );
			CPPAD_ASSERT_UNKNOWN( user_j <= user_n );
			//
			--user_j;
			// argument variables not available during sparsity calculations
			user_x[user_j] = CppAD::numeric_limits<Base>::quiet_NaN();
			// variable index for this argument
			user_ix[user_j] = arg[0];
			//
			if( user_j == 0 )
				user_state = start_user;
			break;

			case UsrrpOp:
			// parameter result for a user atomic function
			CPPAD_ASSERT_NARG_NRES(op, 1, 0);
			CPPAD_ASSERT_UNKNOWN( user_state == ret_user );
			CPPAD_ASSERT_UNKNOWN( user_i <= user_m );
			CPPAD_ASSERT_UNKNOWN( user_j == user_n );
			CPPAD_ASSERT_UNKNOWN( size_t( arg[0] ) < num_par );
			//
			--user_i;
			user_iy[user_i] = 0; // special variable used for parameters
			//
			if( user_i == 0 )
				user_state = arg_user;
			break;

			case UsrrvOp:
			// variable result for a user atomic function
			CPPAD_ASSERT_NARG_NRES(op, 0, 1);
			CPPAD_ASSERT_UNKNOWN( user_state == ret_user );
			CPPAD_ASSERT_UNKNOWN( user_i <= user_m );
			CPPAD_ASSERT_UNKNOWN( user_j == user_n );
			//
			--user_i;
			user_iy[user_i] = i_var; // variable for this result
			//
			if( user_i == 0 )
				user_state = arg_user;
			break;
			// -------------------------------------------------

			case ZmulpvOp:
			CPPAD_ASSERT_NARG_NRES(op, 2, 1);
			reverse_sparse_jacobian_unary_op(
				i_var, arg[1], var_sparsity
			);
			break;
			// -------------------------------------------------

			case ZmulvpOp:
			CPPAD_ASSERT_NARG_NRES(op, 2, 1);
			reverse_sparse_jacobian_unary_op(
				i_var, arg[0], var_sparsity
			);
			break;
			// -------------------------------------------------

			case ZmulvvOp:
			CPPAD_ASSERT_NARG_NRES(op, 2, 1);
			reverse_sparse_jacobian_binary_op(
				i_var, arg, var_sparsity
			);
			break;
			// -------------------------------------------------

			default:
			CPPAD_ASSERT_UNKNOWN(0);
		}
# if CPPAD_REV_JAC_SWEEP_TRACE
		for(j = 0; j < limit; j++)
			z_value[j] = false;
		typename Vector_set::const_iterator itr(var_sparsity, i_var);
		j = *itr;
		while( j < limit )
		{	z_value[j] = true;
			j          = *(++itr);
		}
		printOp(
			std::cout,
			play,
			i_op,
			i_var,
			op,
			arg
		);
		// Note that sparsity for UsrrvOp are computed before call to
		// atomic function so no need to delay printing (as in forward mode)
		if( NumRes(op) > 0 && op != BeginOp ) printOpResult(
			std::cout,
			0,
			(CppAD::vectorBool *) CPPAD_NULL,
			1,
			&z_value
		);
		std::cout << std::endl;
	}
	std::cout << std::endl;
# else
	}
# endif
	// values corresponding to BeginOp
	CPPAD_ASSERT_UNKNOWN( i_op == 0 );
	CPPAD_ASSERT_UNKNOWN( i_var == 0 );

	return;
}
} } // END_CPPAD_LOCAL_NAMESPACE

// preprocessor symbols that are local to this file
# undef CPPAD_REV_JAC_SWEEP_TRACE
# undef CPPAD_ATOMIC_CALL

# endif
