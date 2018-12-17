# ifndef CPPAD_LOCAL_OPTIMIZE_MATCH_OP_HPP
# define CPPAD_LOCAL_OPTIMIZE_MATCH_OP_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-17 Bradley M. Bell

CppAD is distributed under multiple licenses. This distribution is under
the terms of the
                    GNU General Public License Version 3.

A copy of this license is included in the COPYING file of this distribution.
Please visit http://www.coin-or.org/CppAD/ for information on other licenses.
-------------------------------------------------------------------------- */
# include <cppad/local/optimize/hash_code.hpp>
/*!
\file match_op.hpp
Check if current operator matches a previous operator.
*/
// BEGIN_CPPAD_LOCAL_OPTIMIZE_NAMESPACE
namespace CppAD { namespace local { namespace optimize  {
/*!
Search for a previous operator that matches the current one.

If an argument for the current operator is a variable,
and the argument has previous match,
the previous match for the argument is used when checking for a match
for the current operator.

\param play
This is the old operation sequence.

\param opt_op_info
Mapping from operator index to operator information.
The input value of opt_op_info[current].previous is assumed to be zero.
If a match if found,
the output value of opt_op_info[current].previous is set to the
matching operator index, otherwise it is left as is.
Note that opt_op_info[current].previous < current.

\param current
is the index of the current operator which must be an unary
or binary operator. Note that NumArg(ErfOp) == 3 but it is effectivey
a unary operator and is allowed otherwise NumArg( opt_op_info[current].op) < 3.
It is assumed that hash_table_op is initialized as a vector of emtpy
sets. After this initialization, the value of current inceases with
each call to match_op.

\li
This must be a unary or binary
operator; hence, NumArg( opt_op_info[current].op ) is one or two.
There is one exception, NumRes( ErfOp ) == 3, but arg[0]
is the only true arguments (the others are always the same).

\li
This must not be a VecAD load or store operation; i.e.,
LtpvOp, LtvpOp, LtvvOp, StppOp, StpvOp, StvpOp, StvvOp.
It also must not be an independent variable operator InvOp.

\param hash_table_op
is a vector of sets,
hash_table_op.n_set() == CPPAD_HASH_TABLE_SIZE and
hash_table_op.end() == opt_op_info.size().
If i_op is an element of set[j],
then the operation opt_op_info[i_op] has hash code j,
and opt_op_info[i_op] does not match any other element of set[j].
An entry will be added each time match_op is called
and a match for the current operator is not found.
*/
template <class Base>
void match_op(
	const player<Base>*            play           ,
	vector<struct_opt_op_info>&    opt_op_info    ,
	size_t                         current        ,
	sparse_list&                   hash_table_op  )
{	//
	size_t num_op = play->num_op_rec();
	//
	CPPAD_ASSERT_UNKNOWN( num_op == opt_op_info.size() );
	CPPAD_ASSERT_UNKNOWN( opt_op_info[current].previous == 0 );
	CPPAD_ASSERT_UNKNOWN(
		hash_table_op.n_set() == CPPAD_HASH_TABLE_SIZE
	);
	CPPAD_ASSERT_UNKNOWN( hash_table_op.end() == num_op );
	CPPAD_ASSERT_UNKNOWN( current < num_op );
	//
	// current operator
	OpCode        op;
	const addr_t* arg;
	size_t        i_var;
	play->get_op_info(current, op, arg, i_var);
	CPPAD_ASSERT_UNKNOWN( 0 < NumArg(op) );
	CPPAD_ASSERT_UNKNOWN( NumArg(op) <= 3 );
	//
	pod_vector<bool>  variable(3);
	arg_is_variable(op, arg, variable);
	CPPAD_ASSERT_UNKNOWN( variable.size() == 3 );
	//
	// If j-th argument to current operator has a previous operator,
	// this is the j-th argument for previous operator.
	// Otherwise, it is the j-th argument for the current operator.
	addr_t arg_match[3];
	size_t num_arg = NumArg(op);
	for(size_t j = 0; j < num_arg; ++j)
	{	arg_match[j] = arg[j];
		if( variable[j] )
		{	size_t j_op     = play->var2op(arg[j]);
			size_t previous = opt_op_info[j_op].previous;
			if( previous != 0 )
			{	// a previous match, be the end of the line; i.e.,
				// it does not have a previous match.
				CPPAD_ASSERT_UNKNOWN( opt_op_info[previous].previous == 0 );
				//
				OpCode        op_p;
				const addr_t* arg_p;
				size_t        i_var_p;
				play->get_op_info(previous, op_p, arg_p, i_var_p);
				//
				CPPAD_ASSERT_UNKNOWN( NumRes(op_p) > 0 );
				arg_match[j] = addr_t( i_var_p );
			}
		}
	}
	size_t code = optimize_hash_code(op, num_arg, arg_match);
	//
	// iterator for the set with this hash code
	sparse_list_const_iterator itr(hash_table_op, code);
	//
	// check for a match
	size_t count = 0;
	while( *itr != num_op )
	{	++count;
		//
		// candidate previous for current operator
		size_t  candidate  = *itr;
		CPPAD_ASSERT_UNKNOWN( candidate < current );
		CPPAD_ASSERT_UNKNOWN( opt_op_info[candidate].previous == 0 );
		//
		OpCode        op_c;
		const addr_t* arg_c;
		size_t        i_var_c;
		play->get_op_info(candidate, op_c, arg_c, i_var_c);
		//
		// check for a match
		bool match = op == op_c;
		if( match )
		{	for(size_t j = 0; j < num_arg; j++)
			{	if( variable[j] )
				{	size_t previous =
						opt_op_info[ play->var2op(arg_c[j]) ].previous;
					if( previous != 0 )
					{	// must be end of the line for a previous match
						CPPAD_ASSERT_UNKNOWN(
							opt_op_info[previous].previous == 0
						);
						//
						OpCode        op_p;
						const addr_t* arg_p;
						size_t        i_var_p;
						play->get_op_info(previous, op_p, arg_p, i_var_p);
						//
						match &= arg_match[j] == addr_t( i_var_p );
					}
					else match &= arg_match[j] == arg_c[j];
				}
				else
				{	match &= arg_match[j] == arg_c[j];
				}
			}
		}
		if( match )
		{	opt_op_info[current].previous = static_cast<addr_t>( candidate );
			return;
		}
		++itr;
	}

	// special case where operator is commutative
	if( (op == AddvvOp) | (op == MulvvOp ) )
	{	CPPAD_ASSERT_UNKNOWN( NumArg(op) == 2 );
		std::swap( arg_match[0], arg_match[1] );
		//
		code      = optimize_hash_code(op, num_arg, arg_match);
		sparse_list_const_iterator itr_swap(hash_table_op, code);
		while( *itr_swap != num_op )
		{
			size_t candidate  = *itr_swap;
			CPPAD_ASSERT_UNKNOWN( candidate < current );
			CPPAD_ASSERT_UNKNOWN( opt_op_info[candidate].previous == 0 );
			//
			OpCode        op_c;
			const addr_t* arg_c;
			size_t        i_var_c;
			play->get_op_info(candidate, op_c, arg_c, i_var_c);
			//
			bool match = op == op_c;
			if( match )
			{	for(size_t j = 0; j < num_arg; j++)
				{	CPPAD_ASSERT_UNKNOWN( variable[j] )
					size_t previous =
						opt_op_info[ play->var2op(arg_c[j]) ].previous;
					if( previous != 0 )
					{	CPPAD_ASSERT_UNKNOWN(
							opt_op_info[previous].previous == 0
						);
						//
						OpCode        op_p;
						const addr_t* arg_p;
						size_t        i_var_p;
						play->get_op_info(previous, op_p, arg_p, i_var_p);
						//
						match &= arg_match[j] == addr_t( i_var_p );
					}
					else
						match &= arg_match[j] == arg_c[j];
				}
			}
			if( match )
			{	opt_op_info[current].previous = static_cast<addr_t>(candidate);
				return;
			}
			++itr_swap;
		}
	}
	CPPAD_ASSERT_UNKNOWN( count < 11 );
	if( count == 10 )
	{	// restart the list
		hash_table_op.clear(code);
	}
	// no match was found, add this operator the the set for this hash code
	hash_table_op.add_element(code, current);
}

} } } // END_CPPAD_LOCAL_OPTIMIZE_NAMESPACE

# endif
