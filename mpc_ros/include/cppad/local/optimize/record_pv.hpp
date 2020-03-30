# ifndef CPPAD_LOCAL_OPTIMIZE_RECORD_PV_HPP
# define CPPAD_LOCAL_OPTIMIZE_RECORD_PV_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-17 Bradley M. Bell

CppAD is distributed under multiple licenses. This distribution is under
the terms of the
                    GNU General Public License Version 3.

A copy of this license is included in the COPYING file of this distribution.
Please visit http://www.coin-or.org/CppAD/ for information on other licenses.
-------------------------------------------------------------------------- */
/*!
\file record_pv.hpp
Record an operation of the form (parameter op variable).
*/
// BEGIN_CPPAD_LOCAL_OPTIMIZE_NAMESPACE
namespace CppAD { namespace local { namespace optimize  {

/*!
Record an operation of the form (parameter op variable).

\param play
player object corresponding to the old recroding.

\param old2new
mapping from old operator index to information about the new recording.

\param i_op
is the index in the old operation sequence for this operator.
The operator must be one of the following:
AddpvOp, DivpvOp, MulpvOp, PowpvOp, SubpvOp, ZmulpvOp.

\param rec
is the object that will record the new operations.

\return
is the operator and variable indices in the new operation sequence.
*/
template <class Base>
struct_size_pair record_pv(
	const player<Base>*                                play           ,
	const CppAD::vector<struct struct_old2new>&        old2new        ,
	size_t                                             i_op           ,
	recorder<Base>*                                    rec            )
{
	// get_op_info
	OpCode        op;
	const addr_t* arg;
	size_t        i_var;
	play->get_op_info(i_op, op, arg, i_var);
	//
# ifndef NDEBUG
	switch(op)
	{	case AddpvOp:
		case DivpvOp:
		case MulpvOp:
		case PowpvOp:
		case SubpvOp:
		case ZmulpvOp:
		break;

		default:
		CPPAD_ASSERT_UNKNOWN(false);
	}
	// number of parameters corresponding to the old operation sequence.
	size_t npar = play->num_par_rec();
# endif
	//
	// vector of length npar containing the parameters the old operation
	// sequence; i.e., given a parameter index i < npar, the corresponding
	// parameter value is par[i].
	const Base* par = play->GetPar();
	//
	CPPAD_ASSERT_UNKNOWN( NumRes(op) > 0 );
	CPPAD_ASSERT_UNKNOWN( size_t(arg[0]) < npar  );
	CPPAD_ASSERT_UNKNOWN( size_t(arg[1]) < i_var ); // DAG condition
	//
	addr_t new_arg[2];
	new_arg[0]   = rec->PutPar( par[arg[0]] );
	new_arg[1]   = old2new[ play->var2op(arg[1]) ].new_var;
	rec->PutArg( new_arg[0], new_arg[1] );
	//
	struct_size_pair ret;
	ret.i_op  = rec->num_op_rec();
	ret.i_var = rec->PutOp(op);
	CPPAD_ASSERT_UNKNOWN( 0 < new_arg[1] && size_t(new_arg[1]) < ret.i_var );
	return ret;
}

} } } // END_CPPAD_LOCAL_OPTIMIZE_NAMESPACE


# endif
