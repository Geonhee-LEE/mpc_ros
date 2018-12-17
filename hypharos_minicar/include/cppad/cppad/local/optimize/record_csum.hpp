# ifndef CPPAD_LOCAL_OPTIMIZE_RECORD_CSUM_HPP
# define CPPAD_LOCAL_OPTIMIZE_RECORD_CSUM_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-17 Bradley M. Bell

CppAD is distributed under multiple licenses. This distribution is under
the terms of the
                    GNU General Public License Version 3.

A copy of this license is included in the COPYING file of this distribution.
Please visit http://www.coin-or.org/CppAD/ for information on other licenses.
-------------------------------------------------------------------------- */
/*!
\file record_csum.hpp
Recording a cummulative cummulative summation.
*/
# include <cppad/local/optimize/old2new.hpp>

// BEGIN_CPPAD_LOCAL_OPTIMIZE_NAMESPACE
namespace CppAD { namespace local { namespace optimize  {
/*!
Recording a cummulative cummulative summation.

\param play
player object corresponding to the old recroding.

\param opt_op_info
mapping from old index to operator index to operator information

\param old2new
mapping from old operator index to information about the new recording.

\param current
is the index in the old operation sequence for
the variable corresponding to the result for the current operator.
We use the notation i_op = play->var2op(current).
It follows that  NumRes( opt_op_info[i_op].op ) > 0.
If 0 < j_op < i_op, either opt_op_info[j_op].usage == csum_usage,
opt_op_info[j_op].usage = no_usage, or old2new[j_op].new_var != 0.

\param rec
is the object that will record the new operations.

\return
is the operator and variable indices in the new operation sequence.

\param work
Is temporary work space. On input and output,
work.op_stack, work.add_stack, and work.sub_stack, are all empty.
These stacks are passed in so that they are created once
and then be reused with calls to record_csum.

\par Assumptions
opt_op_info[i_o].op
must be one of AddpvOp, AddvvOp, SubpvOp, SubvpOp, SubvvOp.
opt_op_info[i_op].usage != no_usage and ! opt_op_info[i_op].usage == csum_usage.
Furthermore opt_op_info[j_op].usage == csum_usage is true from some
j_op that corresponds to a variable that is an argument to
opt_op_info[i_op].
*/

template <class Base>
struct_size_pair record_csum(
	const player<Base>*                                play           ,
	const vector<struct_opt_op_info>&                  opt_op_info    ,
	const CppAD::vector<struct struct_old2new>&        old2new        ,
	size_t                                             current        ,
	recorder<Base>*                                    rec            ,
	// local information passed so stacks need not be allocated for every call
	struct_csum_stacks&                                work           )
{
# ifndef NDEBUG
	// number of parameters corresponding to the old operation sequence.
	size_t npar = play->num_par_rec();
# endif

	// vector of length npar containing the parameters the old operation
	// sequence; i.e., given a parameter index i < npar, the corresponding
	// parameter value is par[i].
	const Base* par = play->GetPar();

	// check assumption about work space
	CPPAD_ASSERT_UNKNOWN( work.op_stack.empty() );
	CPPAD_ASSERT_UNKNOWN( work.add_stack.empty() );
	CPPAD_ASSERT_UNKNOWN( work.sub_stack.empty() );
	//
	size_t i_op = play->var2op(current);
	CPPAD_ASSERT_UNKNOWN( ! ( opt_op_info[i_op].usage == csum_usage ) );
	//
	// information corresponding to the root node in the cummulative summation
	struct struct_csum_variable var;
	size_t not_used;
	play->get_op_info(i_op, var.op, var.arg, not_used);
	var.add = true;  // was parrent operator positive or negative
	//
	// initialize stack as containing this one operator
	work.op_stack.push( var );
	//
	// initialize sum of parameter values as zero
	Base sum_par(0);
	//
# ifndef NDEBUG
	bool ok = false;
	if( var.op == SubvpOp )
		ok = opt_op_info[ play->var2op(var.arg[0]) ].usage == csum_usage;
	if( var.op == AddpvOp || var.op == SubpvOp )
		ok = opt_op_info[ play->var2op(var.arg[1]) ].usage == csum_usage;
	if( var.op == AddvvOp || var.op == SubvvOp )
	{	ok  = opt_op_info[ play->var2op(var.arg[0]) ].usage == csum_usage;
		ok |= opt_op_info[ play->var2op(var.arg[1]) ].usage == csum_usage;
	}
	CPPAD_ASSERT_UNKNOWN( ok );
# endif
	//
	// while there are operators left on the stack
	while( ! work.op_stack.empty() )
	{	// get this summation operator
		var     = work.op_stack.top();
		work.op_stack.pop();
		OpCode        op      = var.op;
		const addr_t* arg     = var.arg;
		bool          add     = var.add;
		//
		// process first argument to this operator
		switch(op)
		{	// cases where first argument is a parameter
			case AddpvOp:
			case SubpvOp:
			CPPAD_ASSERT_UNKNOWN( size_t(arg[0]) < npar );
			// first argument has same sign as parent node
			if( add )
				sum_par += par[arg[0]];
			else	sum_par -= par[arg[0]];
			break;

			// cases where first argument is a variable
			case AddvvOp:
			case SubvpOp:
			case SubvvOp:
			//
			// check if the first argument has csum usage
			if( opt_op_info[play->var2op(arg[0])].usage == csum_usage )
			{	CPPAD_ASSERT_UNKNOWN(
					size_t( old2new[ play->var2op(arg[0]) ].new_var) == 0
				);
				// push the operator corresponding to the first argument
				size_t i_op_tmp = play->var2op(arg[0]);
				play->get_op_info(i_op_tmp, var.op, var.arg, not_used);
				// first argument has same sign as parent node
				var.add = add;
				work.op_stack.push( var );
			}
			else
			{	// there are no nodes below this one
				CPPAD_ASSERT_UNKNOWN( size_t(arg[0]) < current );
				if( add )
					work.add_stack.push(arg[0]);
				else	work.sub_stack.push(arg[0]);
			}
			break;

			default:
			CPPAD_ASSERT_UNKNOWN(false);
		}
		// process second argument to this operator
		switch(op)
		{	// cases where second argument is a parameter
			case SubvpOp:
			CPPAD_ASSERT_UNKNOWN( size_t(arg[1]) < npar );
			// second argument has opposite sign of parent node
			if( add )
				sum_par -= par[arg[1]];
			else	sum_par += par[arg[1]];
			break;

			// cases where second argument is a variable and has opposite sign
			case SubvvOp:
			case SubpvOp:
			add = ! add;

			// cases where second argument is a variable and has same sign
			case AddvvOp:
			case AddpvOp:
			// check if the second argument has csum usage
			if( opt_op_info[play->var2op(arg[1])].usage == csum_usage )
			{	CPPAD_ASSERT_UNKNOWN(
					size_t( old2new[ play->var2op(arg[1]) ].new_var) == 0
				);
				// push the operator corresoponding to the second arugment
				size_t i_op_tmp = play->var2op(arg[1]);
				play->get_op_info(i_op_tmp, var.op, var.arg, not_used);
				var.add  = add;
				work.op_stack.push( var );
			}
			else
			{	// there are no nodes below this one
				CPPAD_ASSERT_UNKNOWN( size_t(arg[1]) < current );
				if( add )
					work.add_stack.push(arg[1]);
				else	work.sub_stack.push(arg[1]);
			}
			break;

			default:
			CPPAD_ASSERT_UNKNOWN(false);
		}
	}
	// number of variables to add in this cummulative sum operator
	size_t n_add = work.add_stack.size();
	// number of variables to subtract in this cummulative sum operator
	size_t n_sub = work.sub_stack.size();
	//
	CPPAD_ASSERT_UNKNOWN(
		std::numeric_limits<addr_t>::max() >= n_add + n_sub
	);
	//
	rec->PutArg( addr_t(n_add) );                // arg[0]
	rec->PutArg( addr_t(n_sub) );                // arg[1]
	addr_t new_arg = rec->PutPar(sum_par);
	rec->PutArg(new_arg);              // arg[2]
	// addition arguments
	for(size_t i = 0; i < n_add; i++)
	{	CPPAD_ASSERT_UNKNOWN( ! work.add_stack.empty() );
		size_t old_arg = work.add_stack.top();
		new_arg        = old2new[ play->var2op(old_arg) ].new_var;
		CPPAD_ASSERT_UNKNOWN( 0 < new_arg && size_t(new_arg) < current );
		rec->PutArg(new_arg);         // arg[3+i]
		work.add_stack.pop();
	}
	// subtraction arguments
	for(size_t i = 0; i < n_sub; i++)
	{	CPPAD_ASSERT_UNKNOWN( ! work.sub_stack.empty() );
		size_t old_arg = work.sub_stack.top();
		new_arg        = old2new[ play->var2op(old_arg) ].new_var;
		CPPAD_ASSERT_UNKNOWN( 0 < new_arg && size_t(new_arg) < current );
		rec->PutArg(new_arg);      // arg[3 + arg[0] + i]
		work.sub_stack.pop();
	}
	// number of additions plus number of subtractions
	rec->PutArg( addr_t(n_add + n_sub) );      // arg[3 + arg[0] + arg[1]]
	//
	// return value
	struct_size_pair ret;
	ret.i_op  = rec->num_op_rec();
	ret.i_var = rec->PutOp(CSumOp);
	//
	return ret;
}

} } } // END_CPPAD_LOCAL_OPTIMIZE_NAMESPACE


# endif
