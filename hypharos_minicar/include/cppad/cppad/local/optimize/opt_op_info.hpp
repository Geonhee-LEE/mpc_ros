# ifndef CPPAD_LOCAL_OPTIMIZE_OPT_OP_INFO_HPP
# define CPPAD_LOCAL_OPTIMIZE_OPT_OP_INFO_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-17 Bradley M. Bell

CppAD is distributed under multiple licenses. This distribution is under
the terms of the
                    GNU General Public License Version 3.

A copy of this license is included in the COPYING file of this distribution.
Please visit http://www.coin-or.org/CppAD/ for information on other licenses.
-------------------------------------------------------------------------- */

# include <cppad/local/op_code.hpp>
# include <cppad/local/optimize/usage.hpp>

// BEGIN_CPPAD_LOCAL_OPTIMIZE_NAMESPACE
namespace CppAD { namespace local { namespace optimize {

/// information for one operator
struct struct_opt_op_info {
	/*!
	previous operator that can be used in place of this operator.
	\li
	If previous == 0, no such operator was found.
	\li
	If previous != 0,
	opt_op_info[pevious].previous == 0 and
	opt_op_info[previous].usage == yes_usage.
	*/
	addr_t previous;

	/// How is this operator used to compute the dependent variables.
	/// If usage = csum_usage or usage = no_usage, previous = 0.
	enum_usage usage;

};

} } } // END_CPPAD_LOCAL_OPTIMIZE_NAMESPACE
# endif
