# ifndef CPPAD_LOCAL_PLAYER_HPP
# define CPPAD_LOCAL_PLAYER_HPP

/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-17 Bradley M. Bell

CppAD is distributed under multiple licenses. This distribution is under
the terms of the
                    GNU General Public License Version 3.

A copy of this license is included in the COPYING file of this distribution.
Please visit http://www.coin-or.org/CppAD/ for information on other licenses.
-------------------------------------------------------------------------- */

# include <cppad/local/user_state.hpp>
# include <cppad/local/is_pod.hpp>

namespace CppAD { namespace local { // BEGIN_CPPAD_LOCAL_NAMESPACE
/*!
\file player.hpp
File used to define the player class.
*/

/*!
Class used to store and play back an operation sequence recording.

\tparam Base
These were AD< Base > operations when recorded. Operations during playback
are done using the type Base .
*/
template <class Base>
class player {
private:
	// ----------------------------------------------------------------------
	// information that defines the recording

	/// Number of variables in the recording.
	size_t num_var_rec_;

	/// number of vecad load opeations in the reconding
	size_t num_load_op_rec_;

	/// Number of VecAD vectors in the recording
	size_t num_vecad_vec_rec_;

	/// The operators in the recording.
	pod_vector<OpCode> op_vec_;

	/// The operation argument indices in the recording
	pod_vector<addr_t> arg_vec_;

	/// The parameters in the recording.
	/// Note that Base may not be plain old data, so use false in consructor.
	pod_vector<Base> par_vec_;

	/// Character strings ('\\0' terminated) in the recording.
	pod_vector<char> text_vec_;

	/// The VecAD indices in the recording.
	pod_vector<addr_t> vecad_ind_vec_;

	// ----------------------------------------------------------------------
	// Redundant information that simplifies and speeds up iterating through
	// the operation sequence.

	/// index in arg_vec_ corresonding to the first argument for each operator
	pod_vector<addr_t> op2arg_vec_;

	/*!
	Index of the result variable for each operator. If the operator has
	no results, this is not defined. The invalid index num_var_rec_ is used
	when NDEBUG is not defined. If the operator has more than one result, this
	is the primary result; i.e., the last result. Auxillary are only used by
	the operator and not used by other operators.
	*/
	pod_vector<addr_t> op2var_vec_;

	/// Mapping from primary variable index to corresponding operator index.
	/// This is used to traverse sub-graphs of the operation sequence.
	/// This value is valid (invalid) for primary (auxillary) variables.
	pod_vector<addr_t> var2op_vec_;

public:
	// =================================================================
	/// constructor
	player(void) :
	num_var_rec_(0)      ,
	num_load_op_rec_(0)
	{ }

	// =================================================================
	/// destructor
	~player(void)
	{ }

	// ===============================================================
	/*!
	Moving an operation sequence from a recorder to this player

	\param rec
	the object that was used to record the operation sequence. After this
	operation, the state of the recording is no longer defined. For example,
	the pod_vector member variables in this have been swapped with rec.

	\param n_ind
	the number of independent variables (only used for error checking
	when NDEBUG is not defined).

	\par
	Use an assert to check that the length of the following vectors is
	less than the maximum possible value for addr_t; i.e., that an index
	in these vectors can be represented using the type addr_t:
	op_vec_, vecad_ind_vec_, arg_vec_, par_vec_, text_vec_.
	*/
	void get(recorder<Base>& rec, size_t n_ind)
	{
# ifndef NDEBUG
		size_t addr_t_max = size_t( std::numeric_limits<addr_t>::max() );
# endif
		// just set size_t values
		num_var_rec_        = rec.num_var_rec_;
		num_load_op_rec_    = rec.num_load_op_rec_;

		// op_rec_
		op_vec_.swap(rec.op_vec_);
		CPPAD_ASSERT_UNKNOWN(op_vec_.size() < addr_t_max );

		// op_arg_rec_
		arg_vec_.swap(rec.arg_vec_);
		CPPAD_ASSERT_UNKNOWN(arg_vec_.size()    < addr_t_max );

		// par_rec_
		par_vec_.swap(rec.par_vec_);
		CPPAD_ASSERT_UNKNOWN(par_vec_.size() < addr_t_max );

		// text_rec_
		text_vec_.swap(rec.text_vec_);
		CPPAD_ASSERT_UNKNOWN(text_vec_.size() < addr_t_max );

		// vec_ind_rec_
		vecad_ind_vec_.swap(rec.vecad_ind_vec_);
		CPPAD_ASSERT_UNKNOWN(vecad_ind_vec_.size() < addr_t_max );

		// num_vecad_vec_rec_
		num_vecad_vec_rec_ = 0;
		{	// vecad_ind_vec_ contains size of each VecAD followed by
			// the parameter indices used to inialize it.
			size_t i = 0;
			while( i < vecad_ind_vec_.size() )
			{	num_vecad_vec_rec_++;
				i += vecad_ind_vec_[i] + 1;
			}
			CPPAD_ASSERT_UNKNOWN( i == vecad_ind_vec_.size() );
		}

		// op2arg_vec_, op2var_vec_, var2op_vec_
		CPPAD_ASSERT_UNKNOWN( op_vec_[0] == BeginOp );
		CPPAD_ASSERT_NARG_NRES(BeginOp, 1, 1);
		addr_t  num_op    = addr_t( op_vec_.size() );
		addr_t  var_index = 0;
		addr_t  arg_index = 0;
		//
		op2arg_vec_.resize( num_op );
		op2var_vec_.resize( num_op );
		var2op_vec_.resize( num_var_rec_ );
# ifndef NDEBUG
		// value of var2op for auxillary variables is op_vec_.size() (invalid)
		for(size_t i_var = 0; i_var < num_var_rec_; ++i_var)
			var2op_vec_[i_var] = addr_t( op_vec_.size() );
# endif
		for(addr_t i_op = 0; i_op < num_op; ++i_op)
		{	OpCode  op          = op_vec_[i_op];
			//
			// index of first argument for this operator
			op2arg_vec_[i_op]   = arg_index;
			//
			// index of first argument for next operator
			arg_index          += addr_t( NumArg(op) );
			//
			// index of first result for next operator
			var_index          += addr_t( NumRes(op) );
			//
			if( NumRes(op) > 0 )
			{	// index of last (primary) result for this operator
				// when NumRes(op) > 0.
				op2var_vec_[i_op] = var_index - 1;
				//
				// mapping from this primary variable to its operator
				var2op_vec_[var_index - 1] = i_op;
				//
				if( op == CSumOp )
				{	// phony number of arguments
					CPPAD_ASSERT_UNKNOWN( NumArg(CSumOp) == 0 );
					//
					// pointer to first argument for this operator
					addr_t* op_arg = arg_vec_.data() + arg_index;
					//
					// The actual number of arugments for this operator is
					// op_arg[0] + op_arg[1] + 4. Correct index of
					// first argument for next operator
					arg_index += op_arg[0] + op_arg[1] + 4;
				}
			}
			else
			{	// invalid index, no result for this operator
				op2var_vec_[i_op] = addr_t( num_var_rec_ );
				//
				if( op == CSkipOp )
				{	// phony number of arguments
					CPPAD_ASSERT_UNKNOWN( NumArg(CSumOp) == 0 );
					//
					// pointer to first argument for this operator
					addr_t* op_arg = arg_vec_.data() + arg_index;
					//
					// The actual number of arugments for this operator is
					// 7 + op_arg[4] + op_arg[5]. Correct index of
					// first argument for next operator.
					arg_index += 7 + op_arg[4] + op_arg[5];
				}
			}
		}
		check_inv_op(n_ind);
		check_dag();
	}
	// ----------------------------------------------------------------------
	/*!
	Check that InvOp operators start with second operator and are contiguous,
	and there are n_ind of them.
	*/
# ifdef NDEBUG
	void check_inv_op(size_t n_ind)
	{	return; }
# else
	void check_inv_op(size_t n_ind)
	{	size_t num_op = op_vec_.size();
		CPPAD_ASSERT_UNKNOWN( op_vec_[0] = BeginOp );
		CPPAD_ASSERT_UNKNOWN( op_vec_[1] = InvOp   );
		// start at second operator
		for(size_t i_op = 1; i_op < num_op; ++i_op)
		{	OpCode op = op_vec_[i_op];
			CPPAD_ASSERT_UNKNOWN( (op == InvOp) == (i_op <= n_ind) );
		}
		return;
	}

# endif
	// ----------------------------------------------------------------------
	/*!
	Check arguments that are variables, to make sure the have value less
	than or equal to the previously created variable. This is the directed
	acyclic graph condition (DAG).
	*/
# ifdef NDEBUG
	void check_dag(void)
	{	return; }
# else
	void check_dag(void)
	{
		size_t  num_op       = op_vec_.size();
		addr_t arg_var_bound = 0;
		for(size_t i = 0; i < num_op; i++)
		{	OpCode op = op_vec_[i];
			addr_t* op_arg = arg_vec_.data() + op2arg_vec_[i];
			switch(op)
			{
				// cases where nothing to do
				case BeginOp:
				case EndOp:
				case InvOp:
				case LdpOp:
				case ParOp:
				case UserOp:
				case UsrapOp:
				case UsrrpOp:
				case UsrrvOp:
				case StppOp:
				break;

				// only first argument is a variable
				case AbsOp:
				case AcosOp:
				case AcoshOp:
				case AsinOp:
				case AsinhOp:
				case AtanOp:
				case AtanhOp:
				case CosOp:
				case CoshOp:
				case DivvpOp:
				case ErfOp:
				case ExpOp:
				case Expm1Op:
				case LevpOp:
				case LogOp:
				case Log1pOp:
				case LtvpOp:
				case PowvpOp:
				case SignOp:
				case SinOp:
				case SinhOp:
				case SqrtOp:
				case SubvpOp:
				case TanOp:
				case TanhOp:
				case UsravOp:
				case ZmulvpOp:
				CPPAD_ASSERT_UNKNOWN(op_arg[0] <= arg_var_bound );
				break;

				// only second argument is a variable
				case AddpvOp:
				case DisOp:
				case DivpvOp:
				case EqpvOp:
				case LdvOp:
				case LepvOp:
				case LtpvOp:
				case MulpvOp:
				case NepvOp:
				case PowpvOp:
				case StvpOp:
				case SubpvOp:
				case ZmulpvOp:
				CPPAD_ASSERT_UNKNOWN(op_arg[1] <= arg_var_bound );
				break;

				// only first and second arguments are variables
				case AddvvOp:
				case DivvvOp:
				case EqvvOp:
				case LevvOp:
				case LtvvOp:
				case MulvvOp:
				case NevvOp:
				case PowvvOp:
				case SubvvOp:
				case ZmulvvOp:
				CPPAD_ASSERT_UNKNOWN(op_arg[0] <= arg_var_bound );
				CPPAD_ASSERT_UNKNOWN(op_arg[1] <= arg_var_bound );
				break;

				// StpvOp
				case StpvOp:
				CPPAD_ASSERT_UNKNOWN(op_arg[2] <= arg_var_bound );
				break;

				// StvvOp
				case StvvOp:
				CPPAD_ASSERT_UNKNOWN(op_arg[1] <= arg_var_bound );
				CPPAD_ASSERT_UNKNOWN(op_arg[2] <= arg_var_bound );
				break;

				// CSumOp
				case CSumOp:
				{	addr_t num_add = op_arg[0];
					addr_t num_sub = op_arg[1];
					for(addr_t j = 0; j < num_add + num_sub; j++)
						CPPAD_ASSERT_UNKNOWN(op_arg[3+j] <= arg_var_bound);
				}
				break;

				// CExpOp
				case CExpOp:
				if( op_arg[1] & 1 )
					CPPAD_ASSERT_UNKNOWN( op_arg[2] <= arg_var_bound);
				if( op_arg[1] & 2 )
					CPPAD_ASSERT_UNKNOWN( op_arg[3] <= arg_var_bound);
				if( op_arg[1] & 4 )
					CPPAD_ASSERT_UNKNOWN( op_arg[4] <= arg_var_bound);
				if( op_arg[1] & 8 )
					CPPAD_ASSERT_UNKNOWN( op_arg[5] <= arg_var_bound);
				break;

				// PriOp
				case PriOp:
				if( op_arg[0] & 1 )
					CPPAD_ASSERT_UNKNOWN( op_arg[1] <= arg_var_bound);
				if( op_arg[0] & 2 )
					CPPAD_ASSERT_UNKNOWN( op_arg[3] <= arg_var_bound);
				break;

				// CSkipOp, PriOp
				case CSkipOp:
				if( op_arg[1] & 1 )
					CPPAD_ASSERT_UNKNOWN( op_arg[2] <= arg_var_bound);
				if( op_arg[1] & 2 )
					CPPAD_ASSERT_UNKNOWN( op_arg[3] <= arg_var_bound);
				break;

				default:
				CPPAD_ASSERT_UNKNOWN(false);
				break;


			}
			if( NumRes(op) > 0 )
			{	addr_t var_index = op2var_vec_[i];
				if( var_index > 0 )
				{	CPPAD_ASSERT_UNKNOWN(arg_var_bound < var_index);
				}
				else
				{	CPPAD_ASSERT_UNKNOWN(arg_var_bound == var_index);
				}
				//
				arg_var_bound = var_index;
			}
		}
	}
# endif
	// ===============================================================
	/*!
	Copying an operation sequence from another player to this one

	\param play
	the object that contains the operatoion sequence to copy.
	*/
	void operator=(const player& play)
	{
		num_var_rec_        = play.num_var_rec_;
		num_load_op_rec_    = play.num_load_op_rec_;
		op_vec_             = play.op_vec_;
		num_vecad_vec_rec_  = play.num_vecad_vec_rec_;
		vecad_ind_vec_      = play.vecad_ind_vec_;
		arg_vec_            = play.arg_vec_;
		par_vec_            = play.par_vec_;
		text_vec_           = play.text_vec_;
		op2arg_vec_         = play.op2arg_vec_;
		op2var_vec_         = play.op2var_vec_;
		var2op_vec_         = play.var2op_vec_;
	}
	// ===============================================================
	/// Erase the recording stored in the player
	void Erase(void)
	{
		num_var_rec_       = 0;
		num_load_op_rec_   = 0;
		num_vecad_vec_rec_ = 0;

		op_vec_.resize(0);
		vecad_ind_vec_.resize(0);
		arg_vec_.resize(0);
		par_vec_.resize(0);
		text_vec_.resize(0);
		op2arg_vec_.resize(0);
		op2var_vec_.resize(0);
		var2op_vec_.resize(0);
	}
	// ================================================================
	// const functions that retrieve infromation from this player
	// ================================================================
	/*!
	\brief
	fetch the operator corresponding to a primary variable

	\param var_index
	must be the index of a primary variable.

	\return
	is the index of the operator corresponding to this primary variable.
	*/
	size_t var2op(size_t var_index) const
	{	size_t i_op = var2op_vec_[var_index];
		// check that var_index is a primary variable index
		CPPAD_ASSERT_UNKNOWN( i_op < op_vec_.size() );
		return i_op;
	}
	/*!
	\brief
	fetch the information corresponding to an operator

	\param op_index
	index for this operator [in]

	\param op [out]
	op code for this operator.

	\param op_arg [out]
	pointer to the first arguement to this operator.

	\param var_index [out]
	index of the last variable (primary variable) for this operator.
	If there is no primary variable for this operator, i_var not sepcified
	and could have any value.
	*/
	void get_op_info(
		size_t         op_index   ,
		OpCode&        op         ,
		const addr_t*& op_arg     ,
		size_t&        var_index  ) const
	{	op        = OpCode( op_vec_[op_index] );
		op_arg    = op2arg_vec_[op_index] + arg_vec_.data();
		var_index = op2var_vec_[op_index];
		return;
	}
	/*!
	\brief
	unpack extra information corresponding to a UserOp

	\param op [in]
	must be a UserOp

	\param op_arg [in]
	is the arguments for this operator

	\param user_old [out]
	is the extra information passed to the old style user atomic functions.

	\param user_m   [out]
	is the number of results for this user atmoic function.

	\param user_n   [out]
	is the number of arguments for this user atmoic function.

	\return
	Is a pointer to this user atomic function.
	*/
	atomic_base<Base>* get_user_info(
		const OpCode     op         ,
		const addr_t*    op_arg     ,
		size_t&          user_old   ,
		size_t&          user_m     ,
		size_t&          user_n     ) const
	{	atomic_base<Base>* user_atom;
		//
		CPPAD_ASSERT_UNKNOWN( op == UserOp );
		CPPAD_ASSERT_NARG_NRES(op, 4, 0);
		//
		user_old = op_arg[1];
		user_n   = op_arg[2];
		user_m   = op_arg[3];
		CPPAD_ASSERT_UNKNOWN( user_n > 0 );
		//
		size_t user_index = size_t( op_arg[0] );
		user_atom = atomic_base<Base>::class_object(user_index);
# ifndef NDEBUG
		if( user_atom == CPPAD_NULL )
		{	// user_atom is null so cannot use user_atom->afun_name()
			std::string msg = atomic_base<Base>::class_name(user_index)
				+ ": atomic_base function has been deleted";
			CPPAD_ASSERT_KNOWN(false, msg.c_str() );
		}
# endif
		// the atomic_base object corresponding to this user function
		user_atom = atomic_base<Base>::class_object(user_index);
		CPPAD_ASSERT_UNKNOWN( user_atom != CPPAD_NULL );
		return user_atom;
	}
	/*!
	\brief
	fetch an operator from the recording.

	\return
	the i-th operator in the recording.

	\param i
	the index of the operator in recording
	*/
	OpCode GetOp (size_t i) const
	{	return OpCode(op_vec_[i]); }

	/*!
	\brief
	Fetch a VecAD index from the recording.

	\return
	the i-th VecAD index in the recording.

	\param i
	the index of the VecAD index in recording
	*/
	size_t GetVecInd (size_t i) const
	{	return vecad_ind_vec_[i]; }

	/*!
	\brief
	Fetch a parameter from the recording.

	\return
	the i-th parameter in the recording.

	\param i
	the index of the parameter in recording
	*/
	Base GetPar(size_t i) const
	{	return par_vec_[i]; }

	/*!
	\brief
	Fetch entire parameter vector from the recording.

	\return
	the entire parameter vector.

	*/
	const Base* GetPar(void) const
	{	return par_vec_.data(); }

	/*!
	\brief
	Fetch a '\\0' terminated string from the recording.

	\return
	the beginning of the string.

	\param i
	the index where the string begins.
	*/
	const char *GetTxt(size_t i) const
	{	CPPAD_ASSERT_UNKNOWN(i < text_vec_.size() );
		return text_vec_.data() + i;
	}

	/// Fetch number of variables in the recording.
	size_t num_var_rec(void) const
	{	return num_var_rec_; }

	/// Fetch number of vecad load operations
	size_t num_load_op_rec(void) const
	{	return num_load_op_rec_; }

	/// Fetch number of operators in the recording.
	size_t num_op_rec(void) const
	{	return op_vec_.size(); }

	/// Fetch number of VecAD indices in the recording.
	size_t num_vec_ind_rec(void) const
	{	return vecad_ind_vec_.size(); }

	/// Fetch number of VecAD vectors in the recording
	size_t num_vecad_vec_rec(void) const
	{	return num_vecad_vec_rec_; }

	/// Fetch number of argument indices in the recording.
	size_t num_op_arg_rec(void) const
	{	return arg_vec_.size(); }

	/// Fetch number of parameters in the recording.
	size_t num_par_rec(void) const
	{	return par_vec_.size(); }

	/// Fetch number of characters (representing strings) in the recording.
	size_t num_text_rec(void) const
	{	return text_vec_.size(); }

	/// Fetch a rough measure of amount of memory used to store recording
	/// using just lengths, not capacities; see the heading size_op_arg
	/// in the file seq_property.omh.
	size_t Memory(void) const
	{	// check assumptions made by ad_fun<Base>::size_op_seq()
		CPPAD_ASSERT_UNKNOWN( op_vec_.size() == num_op_rec() );
		CPPAD_ASSERT_UNKNOWN( op2arg_vec_.size() == num_op_rec() );
		CPPAD_ASSERT_UNKNOWN( op2var_vec_.size() == num_op_rec() );
		CPPAD_ASSERT_UNKNOWN( arg_vec_.size()    == num_op_arg_rec() );
		CPPAD_ASSERT_UNKNOWN( par_vec_.size() == num_par_rec() );
		CPPAD_ASSERT_UNKNOWN( text_vec_.size() == num_text_rec() );
		CPPAD_ASSERT_UNKNOWN( vecad_ind_vec_.size() == num_vec_ind_rec() );
		//
		return op_vec_.size()        * sizeof(OpCode)
		     + arg_vec_.size()       * sizeof(addr_t)
		     + par_vec_.size()       * sizeof(Base)
		     + text_vec_.size()      * sizeof(char)
		     + vecad_ind_vec_.size() * sizeof(addr_t)
		     + op_vec_.size()        * sizeof(addr_t) * 3
		;
	}

};

} } // END_CPPAD_lOCAL_NAMESPACE
# endif
