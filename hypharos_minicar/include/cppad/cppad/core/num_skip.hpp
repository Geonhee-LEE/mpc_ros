# ifndef CPPAD_CORE_NUM_SKIP_HPP
# define CPPAD_CORE_NUM_SKIP_HPP

/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-17 Bradley M. Bell

CppAD is distributed under multiple licenses. This distribution is under
the terms of the
                    GNU General Public License Version 3.

A copy of this license is included in the COPYING file of this distribution.
Please visit http://www.coin-or.org/CppAD/ for information on other licenses.
-------------------------------------------------------------------------- */

/*
$begin number_skip$$
$spell
	optimizer
	var
	taylor_
$$


$section Number of Variables that Can be Skipped$$
$mindex number_skip$$

$head Syntax$$
$icode%n% = %f%.number_skip()%$$

$subhead See Also$$
$cref seq_property$$

$head Purpose$$
The $cref/conditional expressions/CondExp/$$ use either the
$cref/if_true/CondExp/$$ or $cref/if_false/CondExp/$$.
Hence, some terms only need to be evaluated
depending on the value of the comparison in the conditional expression.
The $cref optimize$$ option is capable of detecting some of these
case and determining variables that can be skipped.
This routine returns the number such variables.

$head n$$
The return value $icode n$$ has type $code size_t$$
is the number of variables that the optimizer has determined can be skipped
(given the independent variable values specified by the previous call to
$cref/f.Forward/Forward/$$ for order zero).

$head f$$
The object $icode f$$ has prototype
$codei%
	ADFun<%Base%> %f%
%$$

$children%
	example/general/number_skip.cpp
%$$
$head Example$$
The file $cref number_skip.cpp$$
contains an example and test of this function.
It returns true if it succeeds and false otherwise.

$end
-----------------------------------------------------------------------------
*/

// BEGIN CppAD namespace
namespace CppAD {

// This routine is not const because it runs through the operations sequence
// 2DO: compute this value during zero order forward operations.
template <typename Base>
size_t ADFun<Base>::number_skip(void)
{	// must pass through operation sequence to map operations to variables
	local::OpCode op;
	size_t        i_op;
	size_t        i_var;
	const addr_t* arg;

	// information defined by forward_user
	size_t user_old=0, user_m=0, user_n=0;

	// number of variables skipped
	size_t num_var_skip = 0;

	// start playback
	i_op = 0;
	play_.get_op_info(i_op, op, arg, i_var);
	CPPAD_ASSERT_UNKNOWN(op == local::BeginOp)
	while(op != local::EndOp)
	{	// next op
		play_.get_op_info(++i_op, op, arg, i_var);
		//
		if( op == local::UserOp )
		{	// skip only appears at front or back UserOp of user atomic call
			bool skip_call = cskip_op_[i_op];
			play_.get_user_info(op, arg, user_old, user_m, user_n);
			CPPAD_ASSERT_UNKNOWN( NumRes(op) == 0 );
			size_t num_op = user_m + user_n + 1;
			for(size_t i = 0; i < num_op; i++)
			{	play_.get_op_info(++i_op, op, arg, i_var);
				if( skip_call )
					num_var_skip += NumRes(op);
			}
			CPPAD_ASSERT_UNKNOWN( op == local::UserOp );
		}
		else
		{	if( cskip_op_[i_op] )
				num_var_skip += NumRes(op);
		}
	}
	return num_var_skip;
}

} // END CppAD namespace


# endif
