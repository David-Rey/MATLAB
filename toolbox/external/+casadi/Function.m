classdef  Function < casadi.SharedObject & casadi.PrintableCommon
    %FUNCTION [INTERNAL] 
    %
    %
    % Function object.
    %
    %A  Function instance is a general multiple-input, multiple-output function 
    %where 
    %each input and output can be a sparse matrix.
    % For an introduction to
    % this class, see the CasADi user guide. Function is a reference counted and 
    %immutable class; copying a class instance 
    %is very cheap and its behavior 
    %(with some exceptions) is not affected 
    %by calling its member functions.
    %
    %Joel Andersson
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1uw
    %
    %>List of available options
    %
    %+------------------+-----------------+------------------+------------------+
    %|        Id        |      Type       |   Description    |     Used in      |
    %+==================+=================+==================+==================+
    %| ad_weight        | OT_DOUBLE       | Weighting factor | casadi::Function |
    %|                  |                 | for derivative   | Internal         |
    %|                  |                 | calculation.When |                  |
    %|                  |                 | there is an      |                  |
    %|                  |                 | option of either |                  |
    %|                  |                 | using forward or |                  |
    %|                  |                 | reverse mode     |                  |
    %|                  |                 | directional      |                  |
    %|                  |                 | derivatives, the |                  |
    %|                  |                 | condition ad_wei |                  |
    %|                  |                 | ght*nf<=(1-      |                  |
    %|                  |                 | ad_weight)*na is |                  |
    %|                  |                 | used where nf    |                  |
    %|                  |                 | and na are       |                  |
    %|                  |                 | estimates of the |                  |
    %|                  |                 | number of        |                  |
    %|                  |                 | forward/reverse  |                  |
    %|                  |                 | mode directional |                  |
    %|                  |                 | derivatives      |                  |
    %|                  |                 | needed. By       |                  |
    %|                  |                 | default,         |                  |
    %|                  |                 | ad_weight is     |                  |
    %|                  |                 | calculated       |                  |
    %|                  |                 | automatically,   |                  |
    %|                  |                 | but this can be  |                  |
    %|                  |                 | overridden by    |                  |
    %|                  |                 | setting this     |                  |
    %|                  |                 | option. In       |                  |
    %|                  |                 | particular, 0    |                  |
    %|                  |                 | means forcing    |                  |
    %|                  |                 | forward mode and |                  |
    %|                  |                 | 1 forcing        |                  |
    %|                  |                 | reverse mode.    |                  |
    %|                  |                 | Leave unset for  |                  |
    %|                  |                 | (class specific) |                  |
    %|                  |                 | heuristics.      |                  |
    %+------------------+-----------------+------------------+------------------+
    %| ad_weight_sp     | OT_DOUBLE       | Weighting factor | casadi::Function |
    %|                  |                 | for sparsity     | Internal         |
    %|                  |                 | pattern          |                  |
    %|                  |                 | calculation calc |                  |
    %|                  |                 | ulation.Override |                  |
    %|                  |                 | s default        |                  |
    %|                  |                 | behavior. Set to |                  |
    %|                  |                 | 0 and 1 to force |                  |
    %|                  |                 | forward and      |                  |
    %|                  |                 | reverse mode     |                  |
    %|                  |                 | respectively.    |                  |
    %|                  |                 | Cf. option       |                  |
    %|                  |                 | "ad_weight".     |                  |
    %|                  |                 | When set to -1,  |                  |
    %|                  |                 | sparsity is      |                  |
    %|                  |                 | completely       |                  |
    %|                  |                 | ignored and      |                  |
    %|                  |                 | dense matrices   |                  |
    %|                  |                 | are used.        |                  |
    %+------------------+-----------------+------------------+------------------+
    %| always_inline    | OT_BOOL         | Force inlining.  | casadi::Function |
    %|                  |                 |                  | Internal         |
    %+------------------+-----------------+------------------+------------------+
    %| cache            | OT_DICT         | Prepopulate the  | casadi::Function |
    %|                  |                 | function cache.  | Internal         |
    %|                  |                 | Default: empty   |                  |
    %+------------------+-----------------+------------------+------------------+
    %| compiler         | OT_STRING       | Just-in-time     | casadi::Function |
    %|                  |                 | compiler plugin  | Internal         |
    %|                  |                 | to be used.      |                  |
    %+------------------+-----------------+------------------+------------------+
    %| custom_jacobian  | OT_FUNCTION     | Override         | casadi::Function |
    %|                  |                 | CasADi's AD. Use | Internal         |
    %|                  |                 | together with    |                  |
    %|                  |                 | 'jac_penalty':   |                  |
    %|                  |                 | 0. Note: Highly  |                  |
    %|                  |                 | experimental.    |                  |
    %|                  |                 | Syntax may break |                  |
    %|                  |                 | often.           |                  |
    %+------------------+-----------------+------------------+------------------+
    %| der_options      | OT_DICT         | Default options  | casadi::Function |
    %|                  |                 | to be used to    | Internal         |
    %|                  |                 | populate         |                  |
    %|                  |                 | forward_options, |                  |
    %|                  |                 | reverse_options, |                  |
    %|                  |                 | and              |                  |
    %|                  |                 | jacobian_options |                  |
    %|                  |                 | before those     |                  |
    %|                  |                 | options are      |                  |
    %|                  |                 | merged in.       |                  |
    %+------------------+-----------------+------------------+------------------+
    %| derivative_of    | OT_FUNCTION     | The function is  | casadi::Function |
    %|                  |                 | a derivative of  | Internal         |
    %|                  |                 | another          |                  |
    %|                  |                 | function. The    |                  |
    %|                  |                 | type of          |                  |
    %|                  |                 | derivative       |                  |
    %|                  |                 | (directional     |                  |
    %|                  |                 | derivative,      |                  |
    %|                  |                 | Jacobian) is     |                  |
    %|                  |                 | inferred from    |                  |
    %|                  |                 | the function     |                  |
    %|                  |                 | name.            |                  |
    %+------------------+-----------------+------------------+------------------+
    %| dump             | OT_BOOL         | Dump function to | casadi::Function |
    %|                  |                 | file upon first  | Internal         |
    %|                  |                 | evaluation.      |                  |
    %|                  |                 | [false]          |                  |
    %+------------------+-----------------+------------------+------------------+
    %| dump_dir         | OT_STRING       | Directory to     | casadi::Function |
    %|                  |                 | dump             | Internal         |
    %|                  |                 | inputs/outputs   |                  |
    %|                  |                 | to. Make sure    |                  |
    %|                  |                 | the directory    |                  |
    %|                  |                 | exists [.]       |                  |
    %+------------------+-----------------+------------------+------------------+
    %| dump_format      | OT_STRING       | Choose file      | casadi::Function |
    %|                  |                 | format to dump   | Internal         |
    %|                  |                 | matrices. See    |                  |
    %|                  |                 | DM.from_file     |                  |
    %|                  |                 | [mtx]            |                  |
    %+------------------+-----------------+------------------+------------------+
    %| dump_in          | OT_BOOL         | Dump numerical   | casadi::Function |
    %|                  |                 | values of inputs | Internal         |
    %|                  |                 | to file          |                  |
    %|                  |                 | (readable with   |                  |
    %|                  |                 | DM.from_file )   |                  |
    %|                  |                 | [default: false] |                  |
    %+------------------+-----------------+------------------+------------------+
    %| dump_out         | OT_BOOL         | Dump numerical   | casadi::Function |
    %|                  |                 | values of        | Internal         |
    %|                  |                 | outputs to file  |                  |
    %|                  |                 | (readable with   |                  |
    %|                  |                 | DM.from_file )   |                  |
    %|                  |                 | [default: false] |                  |
    %+------------------+-----------------+------------------+------------------+
    %| enable_fd        | OT_BOOL         | Enable           | casadi::Function |
    %|                  |                 | derivative       | Internal         |
    %|                  |                 | calculation by   |                  |
    %|                  |                 | finite           |                  |
    %|                  |                 | differencing.    |                  |
    %|                  |                 | [default:        |                  |
    %|                  |                 | false]]          |                  |
    %+------------------+-----------------+------------------+------------------+
    %| enable_forward   | OT_BOOL         | Enable           | casadi::Function |
    %|                  |                 | derivative       | Internal         |
    %|                  |                 | calculation      |                  |
    %|                  |                 | using generated  |                  |
    %|                  |                 | functions for    |                  |
    %|                  |                 | Jacobian-times-  |                  |
    %|                  |                 | vector products  |                  |
    %|                  |                 | - typically      |                  |
    %|                  |                 | using forward    |                  |
    %|                  |                 | mode AD - if     |                  |
    %|                  |                 | available.       |                  |
    %|                  |                 | [default: true]  |                  |
    %+------------------+-----------------+------------------+------------------+
    %| enable_jacobian  | OT_BOOL         | Enable           | casadi::Function |
    %|                  |                 | derivative       | Internal         |
    %|                  |                 | calculation      |                  |
    %|                  |                 | using generated  |                  |
    %|                  |                 | functions for    |                  |
    %|                  |                 | Jacobians of all |                  |
    %|                  |                 | differentiable   |                  |
    %|                  |                 | outputs with     |                  |
    %|                  |                 | respect to all   |                  |
    %|                  |                 | differentiable   |                  |
    %|                  |                 | inputs - if      |                  |
    %|                  |                 | available.       |                  |
    %|                  |                 | [default: true]  |                  |
    %+------------------+-----------------+------------------+------------------+
    %| enable_reverse   | OT_BOOL         | Enable           | casadi::Function |
    %|                  |                 | derivative       | Internal         |
    %|                  |                 | calculation      |                  |
    %|                  |                 | using generated  |                  |
    %|                  |                 | functions for    |                  |
    %|                  |                 | transposed       |                  |
    %|                  |                 | Jacobian-times-  |                  |
    %|                  |                 | vector products  |                  |
    %|                  |                 | - typically      |                  |
    %|                  |                 | using reverse    |                  |
    %|                  |                 | mode AD - if     |                  |
    %|                  |                 | available.       |                  |
    %|                  |                 | [default: true]  |                  |
    %+------------------+-----------------+------------------+------------------+
    %| error_on_fail    | OT_BOOL         | Throw exceptions | casadi::ProtoFun |
    %|                  |                 | when function    | ction            |
    %|                  |                 | evaluation fails |                  |
    %|                  |                 | (default true).  |                  |
    %+------------------+-----------------+------------------+------------------+
    %| external_transfo | OT_VECTORVECTOR | List of external | casadi::Function |
    %| rm               |                 | _transform       | Internal         |
    %|                  |                 | instruction      |                  |
    %|                  |                 | arguments.       |                  |
    %|                  |                 | Default: empty   |                  |
    %+------------------+-----------------+------------------+------------------+
    %| fd_method        | OT_STRING       | Method for       | casadi::Function |
    %|                  |                 | finite           | Internal         |
    %|                  |                 | differencing     |                  |
    %|                  |                 | [default         |                  |
    %|                  |                 | 'central']       |                  |
    %+------------------+-----------------+------------------+------------------+
    %| fd_options       | OT_DICT         | Options to be    | casadi::Function |
    %|                  |                 | passed to the    | Internal         |
    %|                  |                 | finite           |                  |
    %|                  |                 | difference       |                  |
    %|                  |                 | instance         |                  |
    %+------------------+-----------------+------------------+------------------+
    %| forward_options  | OT_DICT         | Options to be    | casadi::Function |
    %|                  |                 | passed to a      | Internal         |
    %|                  |                 | forward mode     |                  |
    %|                  |                 | constructor      |                  |
    %+------------------+-----------------+------------------+------------------+
    %| gather_stats     | OT_BOOL         | Deprecated       | casadi::Function |
    %|                  |                 | option           | Internal         |
    %|                  |                 | (ignored):       |                  |
    %|                  |                 | Statistics are   |                  |
    %|                  |                 | now always       |                  |
    %|                  |                 | collected.       |                  |
    %+------------------+-----------------+------------------+------------------+
    %| input_scheme     | OT_STRINGVECTOR | Deprecated       | casadi::Function |
    %|                  |                 | option (ignored) | Internal         |
    %+------------------+-----------------+------------------+------------------+
    %| inputs_check     | OT_BOOL         | Throw exceptions | casadi::Function |
    %|                  |                 | when the         | Internal         |
    %|                  |                 | numerical values |                  |
    %|                  |                 | of the inputs    |                  |
    %|                  |                 | don't make sense |                  |
    %+------------------+-----------------+------------------+------------------+
    %| is_diff_in       | OT_BOOLVECTOR   | Indicate for     | casadi::Function |
    %|                  |                 | each input if it | Internal         |
    %|                  |                 | should be        |                  |
    %|                  |                 | differentiable.  |                  |
    %+------------------+-----------------+------------------+------------------+
    %| is_diff_out      | OT_BOOLVECTOR   | Indicate for     | casadi::Function |
    %|                  |                 | each output if   | Internal         |
    %|                  |                 | it should be     |                  |
    %|                  |                 | differentiable.  |                  |
    %+------------------+-----------------+------------------+------------------+
    %| jac_penalty      | OT_DOUBLE       | When requested   | casadi::Function |
    %|                  |                 | for a number of  | Internal         |
    %|                  |                 | forward/reverse  |                  |
    %|                  |                 | directions, it   |                  |
    %|                  |                 | may be cheaper   |                  |
    %|                  |                 | to compute first |                  |
    %|                  |                 | the full         |                  |
    %|                  |                 | jacobian and     |                  |
    %|                  |                 | then multiply    |                  |
    %|                  |                 | with seeds,      |                  |
    %|                  |                 | rather than      |                  |
    %|                  |                 | obtain the       |                  |
    %|                  |                 | requested        |                  |
    %|                  |                 | directions in a  |                  |
    %|                  |                 | straightforward  |                  |
    %|                  |                 | manner. Casadi   |                  |
    %|                  |                 | uses a heuristic |                  |
    %|                  |                 | to decide which  |                  |
    %|                  |                 | is cheaper. A    |                  |
    %|                  |                 | high value of    |                  |
    %|                  |                 | 'jac_penalty'    |                  |
    %|                  |                 | makes it less    |                  |
    %|                  |                 | likely for the   |                  |
    %|                  |                 | heurstic to      |                  |
    %|                  |                 | chose the full   |                  |
    %|                  |                 | Jacobian         |                  |
    %|                  |                 | strategy. The    |                  |
    %|                  |                 | special value -1 |                  |
    %|                  |                 | indicates never  |                  |
    %|                  |                 | to use the full  |                  |
    %|                  |                 | Jacobian         |                  |
    %|                  |                 | strategy         |                  |
    %+------------------+-----------------+------------------+------------------+
    %| jacobian_options | OT_DICT         | Options to be    | casadi::Function |
    %|                  |                 | passed to a      | Internal         |
    %|                  |                 | Jacobian         |                  |
    %|                  |                 | constructor      |                  |
    %+------------------+-----------------+------------------+------------------+
    %| jit              | OT_BOOL         | Use just-in-time | casadi::Function |
    %|                  |                 | compiler to      | Internal         |
    %|                  |                 | speed up the     |                  |
    %|                  |                 | evaluation       |                  |
    %+------------------+-----------------+------------------+------------------+
    %| jit_cleanup      | OT_BOOL         | Cleanup up the   | casadi::Function |
    %|                  |                 | temporary source | Internal         |
    %|                  |                 | file that jit    |                  |
    %|                  |                 | creates.         |                  |
    %|                  |                 | Default: true    |                  |
    %+------------------+-----------------+------------------+------------------+
    %| jit_name         | OT_STRING       | The file name    | casadi::Function |
    %|                  |                 | used to write    | Internal         |
    %|                  |                 | out code. The    |                  |
    %|                  |                 | actual file      |                  |
    %|                  |                 | names used       |                  |
    %|                  |                 | depend on 'jit_t |                  |
    %|                  |                 | emp_suffix' and  |                  |
    %|                  |                 | include          |                  |
    %|                  |                 | extensions.      |                  |
    %|                  |                 | Default:         |                  |
    %|                  |                 | 'jit_tmp'        |                  |
    %+------------------+-----------------+------------------+------------------+
    %| jit_options      | OT_DICT         | Options to be    | casadi::Function |
    %|                  |                 | passed to the    | Internal         |
    %|                  |                 | jit compiler.    |                  |
    %+------------------+-----------------+------------------+------------------+
    %| jit_serialize    | OT_STRING       | Specify          | casadi::Function |
    %|                  |                 | behaviour when   | Internal         |
    %|                  |                 | serializing a    |                  |
    %|                  |                 | jitted function: |                  |
    %|                  |                 | SOURCE|link|embe |                  |
    %|                  |                 | d.               |                  |
    %+------------------+-----------------+------------------+------------------+
    %| jit_temp_suffix  | OT_BOOL         | Use a temporary  | casadi::Function |
    %|                  |                 | (seemingly       | Internal         |
    %|                  |                 | random) filename |                  |
    %|                  |                 | suffix for       |                  |
    %|                  |                 | generated code   |                  |
    %|                  |                 | and libraries.   |                  |
    %|                  |                 | This is desired  |                  |
    %|                  |                 | for thread-      |                  |
    %|                  |                 | safety. This     |                  |
    %|                  |                 | behaviour may    |                  |
    %|                  |                 | defeat caching   |                  |
    %|                  |                 | compiler         |                  |
    %|                  |                 | wrappers.        |                  |
    %|                  |                 | Default: true    |                  |
    %+------------------+-----------------+------------------+------------------+
    %| max_io           | OT_INT          | Acceptable       | casadi::Function |
    %|                  |                 | number of inputs | Internal         |
    %|                  |                 | and outputs.     |                  |
    %|                  |                 | Warn if          |                  |
    %|                  |                 | exceeded.        |                  |
    %+------------------+-----------------+------------------+------------------+
    %| max_num_dir      | OT_INT          | Specify the      | casadi::Function |
    %|                  |                 | maximum number   | Internal         |
    %|                  |                 | of directions    |                  |
    %|                  |                 | for derivative   |                  |
    %|                  |                 | functions.       |                  |
    %|                  |                 | Overrules the    |                  |
    %|                  |                 | builtin optimize |                  |
    %|                  |                 | d_num_dir.       |                  |
    %+------------------+-----------------+------------------+------------------+
    %| never_inline     | OT_BOOL         | Forbid inlining. | casadi::Function |
    %|                  |                 |                  | Internal         |
    %+------------------+-----------------+------------------+------------------+
    %| output_scheme    | OT_STRINGVECTOR | Deprecated       | casadi::Function |
    %|                  |                 | option (ignored) | Internal         |
    %+------------------+-----------------+------------------+------------------+
    %| post_expand      | OT_BOOL         | After            | casadi::Function |
    %|                  |                 | construction,    | Internal         |
    %|                  |                 | expand this      |                  |
    %|                  |                 | Function .       |                  |
    %|                  |                 | Default: False   |                  |
    %+------------------+-----------------+------------------+------------------+
    %| post_expand_opti | OT_DICT         | Options to be    | casadi::Function |
    %| ons              |                 | passed to post-  | Internal         |
    %|                  |                 | construction     |                  |
    %|                  |                 | expansion.       |                  |
    %|                  |                 | Default: empty   |                  |
    %+------------------+-----------------+------------------+------------------+
    %| print_in         | OT_BOOL         | Print numerical  | casadi::Function |
    %|                  |                 | values of inputs | Internal         |
    %|                  |                 | [default: false] |                  |
    %+------------------+-----------------+------------------+------------------+
    %| print_out        | OT_BOOL         | Print numerical  | casadi::Function |
    %|                  |                 | values of        | Internal         |
    %|                  |                 | outputs          |                  |
    %|                  |                 | [default: false] |                  |
    %+------------------+-----------------+------------------+------------------+
    %| print_time       | OT_BOOL         | print            | casadi::ProtoFun |
    %|                  |                 | information      | ction            |
    %|                  |                 | about execution  |                  |
    %|                  |                 | time. Implies    |                  |
    %|                  |                 | record_time.     |                  |
    %+------------------+-----------------+------------------+------------------+
    %| record_time      | OT_BOOL         | record           | casadi::ProtoFun |
    %|                  |                 | information      | ction            |
    %|                  |                 | about execution  |                  |
    %|                  |                 | time, for        |                  |
    %|                  |                 | retrieval with   |                  |
    %|                  |                 | stats() .        |                  |
    %+------------------+-----------------+------------------+------------------+
    %| regularity_check | OT_BOOL         | Throw exceptions | casadi::ProtoFun |
    %|                  |                 | when NaN or Inf  | ction            |
    %|                  |                 | appears during   |                  |
    %|                  |                 | evaluation       |                  |
    %+------------------+-----------------+------------------+------------------+
    %| reverse_options  | OT_DICT         | Options to be    | casadi::Function |
    %|                  |                 | passed to a      | Internal         |
    %|                  |                 | reverse mode     |                  |
    %|                  |                 | constructor      |                  |
    %+------------------+-----------------+------------------+------------------+
    %| user_data        | OT_VOIDPTR      | A user-defined   | casadi::Function |
    %|                  |                 | field that can   | Internal         |
    %|                  |                 | be used to       |                  |
    %|                  |                 | identify the     |                  |
    %|                  |                 | function or pass |                  |
    %|                  |                 | additional       |                  |
    %|                  |                 | information      |                  |
    %+------------------+-----------------+------------------+------------------+
    %| verbose          | OT_BOOL         | Verbose          | casadi::ProtoFun |
    %|                  |                 | evaluation  for  | ction            |
    %|                  |                 | debugging        |                  |
    %+------------------+-----------------+------------------+------------------+
    %
    %C++ includes: function.hpp
    %
    %
  methods
    function this = swig_this(self)
      this = casadiMEX(3, self);
    end
    function delete(self)
        if self.swigPtr
          casadiMEX(693, self);
          self.SwigClear();
        end
    end
    function varargout = expand(self,varargin)
    %EXPAND [INTERNAL] 
    %
    %  Function = EXPAND(self)
    %  Function = EXPAND(self, char name, struct opts)
    %
    %Expand a function to SX.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1v5
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L207
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L317-L333
    %
    %
    %
    %.......
    %
    %::
    %
    %  EXPAND(self)
    %
    %
    %
    %[INTERNAL] 
    %Expand a function to SX.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1v5
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L206
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L307-L315
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  EXPAND(self, char name, struct opts)
    %
    %
    %
    %[INTERNAL] 
    %Expand a function to SX.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1v5
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L207
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L317-L333
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(694, self, varargin{:});
    end
    function varargout = n_in(self,varargin)
    %N_IN [INTERNAL] 
    %
    %  int = N_IN(self)
    %
    %Get the number of function inputs.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1v8
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L228
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L818-L820
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(695, self, varargin{:});
    end
    function varargout = n_out(self,varargin)
    %N_OUT [INTERNAL] 
    %
    %  int = N_OUT(self)
    %
    %Get the number of function outputs.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1v9
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L233
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L822-L824
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(696, self, varargin{:});
    end
    function varargout = size1_in(self,varargin)
    %SIZE1_IN [INTERNAL] 
    %
    %  int = SIZE1_IN(self, int ind)
    %  int = SIZE1_IN(self, char iname)
    %
    %Get input dimension.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1va
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L240
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L240-L240
    %
    %
    %
    %.......
    %
    %::
    %
    %  SIZE1_IN(self, int ind)
    %
    %
    %
    %[INTERNAL] 
    %Get input dimension.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1va
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L239
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L826-L828
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  SIZE1_IN(self, char iname)
    %
    %
    %
    %[INTERNAL] 
    %Get input dimension.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1va
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L240
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L240-L240
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(697, self, varargin{:});
    end
    function varargout = size2_in(self,varargin)
    %SIZE2_IN [INTERNAL] 
    %
    %  int = SIZE2_IN(self, int ind)
    %  int = SIZE2_IN(self, char iname)
    %
    %Get input dimension.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1va
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L242
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L242-L242
    %
    %
    %
    %.......
    %
    %::
    %
    %  SIZE2_IN(self, int ind)
    %
    %
    %
    %[INTERNAL] 
    %Get input dimension.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1va
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L241
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L830-L832
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  SIZE2_IN(self, char iname)
    %
    %
    %
    %[INTERNAL] 
    %Get input dimension.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1va
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L242
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L242-L242
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(698, self, varargin{:});
    end
    function varargout = size_in(self,varargin)
    %SIZE_IN [INTERNAL] 
    %
    %  [int,int] = SIZE_IN(self, int ind)
    %  [int,int] = SIZE_IN(self, char iname)
    %
    %Get input dimension.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1va
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L244
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L244-L246
    %
    %
    %
    %.......
    %
    %::
    %
    %  SIZE_IN(self, int ind)
    %
    %
    %
    %[INTERNAL] 
    %Get input dimension.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1va
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L243
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L842-L844
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  SIZE_IN(self, char iname)
    %
    %
    %
    %[INTERNAL] 
    %Get input dimension.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1va
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L244
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L244-L246
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(699, self, varargin{:});
    end
    function varargout = size1_out(self,varargin)
    %SIZE1_OUT [INTERNAL] 
    %
    %  int = SIZE1_OUT(self, int ind)
    %  int = SIZE1_OUT(self, char oname)
    %
    %Get output dimension.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vb
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L254
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L254-L254
    %
    %
    %
    %.......
    %
    %::
    %
    %  SIZE1_OUT(self, int ind)
    %
    %
    %
    %[INTERNAL] 
    %Get output dimension.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vb
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L253
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L834-L836
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  SIZE1_OUT(self, char oname)
    %
    %
    %
    %[INTERNAL] 
    %Get output dimension.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vb
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L254
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L254-L254
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(700, self, varargin{:});
    end
    function varargout = size2_out(self,varargin)
    %SIZE2_OUT [INTERNAL] 
    %
    %  int = SIZE2_OUT(self, int ind)
    %  int = SIZE2_OUT(self, char oname)
    %
    %Get output dimension.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vb
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L256
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L256-L256
    %
    %
    %
    %.......
    %
    %::
    %
    %  SIZE2_OUT(self, int ind)
    %
    %
    %
    %[INTERNAL] 
    %Get output dimension.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vb
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L255
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L838-L840
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  SIZE2_OUT(self, char oname)
    %
    %
    %
    %[INTERNAL] 
    %Get output dimension.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vb
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L256
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L256-L256
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(701, self, varargin{:});
    end
    function varargout = size_out(self,varargin)
    %SIZE_OUT [INTERNAL] 
    %
    %  [int,int] = SIZE_OUT(self, int ind)
    %  [int,int] = SIZE_OUT(self, char oname)
    %
    %Get output dimension.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vb
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L258
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L258-L260
    %
    %
    %
    %.......
    %
    %::
    %
    %  SIZE_OUT(self, int ind)
    %
    %
    %
    %[INTERNAL] 
    %Get output dimension.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vb
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L257
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L846-L848
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  SIZE_OUT(self, char oname)
    %
    %
    %
    %[INTERNAL] 
    %Get output dimension.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vb
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L258
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L258-L260
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(702, self, varargin{:});
    end
    function varargout = nnz_in(self,varargin)
    %NNZ_IN [INTERNAL] 
    %
    %  int = NNZ_IN(self)
    %  int = NNZ_IN(self, int ind)
    %  int = NNZ_IN(self, char iname)
    %
    %Get number of input nonzeros.
    %
    %For a particular input or for all of the inputs
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vc
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L271
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L271-L271
    %
    %
    %
    %.......
    %
    %::
    %
    %  NNZ_IN(self)
    %
    %
    %
    %[INTERNAL] 
    %Get number of input nonzeros.
    %
    %For a particular input or for all of the inputs
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vc
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L269
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L850-L852
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  NNZ_IN(self, int ind)
    %
    %
    %
    %[INTERNAL] 
    %Get number of input nonzeros.
    %
    %For a particular input or for all of the inputs
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vc
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L270
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L866-L868
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  NNZ_IN(self, char iname)
    %
    %
    %
    %[INTERNAL] 
    %Get number of input nonzeros.
    %
    %For a particular input or for all of the inputs
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vc
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L271
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L271-L271
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(703, self, varargin{:});
    end
    function varargout = nnz_out(self,varargin)
    %NNZ_OUT [INTERNAL] 
    %
    %  int = NNZ_OUT(self)
    %  int = NNZ_OUT(self, int ind)
    %  int = NNZ_OUT(self, char oname)
    %
    %Get number of output nonzeros.
    %
    %For a particular output or for all of the outputs
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vd
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L282
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L282-L282
    %
    %
    %
    %.......
    %
    %::
    %
    %  NNZ_OUT(self)
    %
    %
    %
    %[INTERNAL] 
    %Get number of output nonzeros.
    %
    %For a particular output or for all of the outputs
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vd
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L280
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L854-L856
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  NNZ_OUT(self, int ind)
    %
    %
    %
    %[INTERNAL] 
    %Get number of output nonzeros.
    %
    %For a particular output or for all of the outputs
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vd
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L281
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L870-L872
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  NNZ_OUT(self, char oname)
    %
    %
    %
    %[INTERNAL] 
    %Get number of output nonzeros.
    %
    %For a particular output or for all of the outputs
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vd
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L282
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L282-L282
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(704, self, varargin{:});
    end
    function varargout = numel_in(self,varargin)
    %NUMEL_IN [INTERNAL] 
    %
    %  int = NUMEL_IN(self)
    %  int = NUMEL_IN(self, int ind)
    %  int = NUMEL_IN(self, char iname)
    %
    %Get number of input elements.
    %
    %For a particular input or for all of the inputs
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1ve
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L293
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L293-L293
    %
    %
    %
    %.......
    %
    %::
    %
    %  NUMEL_IN(self)
    %
    %
    %
    %[INTERNAL] 
    %Get number of input elements.
    %
    %For a particular input or for all of the inputs
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1ve
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L291
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L858-L860
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  NUMEL_IN(self, int ind)
    %
    %
    %
    %[INTERNAL] 
    %Get number of input elements.
    %
    %For a particular input or for all of the inputs
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1ve
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L292
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L874-L876
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  NUMEL_IN(self, char iname)
    %
    %
    %
    %[INTERNAL] 
    %Get number of input elements.
    %
    %For a particular input or for all of the inputs
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1ve
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L293
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L293-L293
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(705, self, varargin{:});
    end
    function varargout = numel_out(self,varargin)
    %NUMEL_OUT [INTERNAL] 
    %
    %  int = NUMEL_OUT(self)
    %  int = NUMEL_OUT(self, int ind)
    %  int = NUMEL_OUT(self, char oname)
    %
    %Get number of output elements.
    %
    %For a particular output or for all of the outputs
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vf
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L304
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L304-L304
    %
    %
    %
    %.......
    %
    %::
    %
    %  NUMEL_OUT(self)
    %
    %
    %
    %[INTERNAL] 
    %Get number of output elements.
    %
    %For a particular output or for all of the outputs
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vf
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L302
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L862-L864
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  NUMEL_OUT(self, int ind)
    %
    %
    %
    %[INTERNAL] 
    %Get number of output elements.
    %
    %For a particular output or for all of the outputs
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vf
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L303
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L878-L880
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  NUMEL_OUT(self, char oname)
    %
    %
    %
    %[INTERNAL] 
    %Get number of output elements.
    %
    %For a particular output or for all of the outputs
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vf
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L304
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L304-L304
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(706, self, varargin{:});
    end
    function varargout = name_in(self,varargin)
    %NAME_IN [INTERNAL] 
    %
    %  {char} = NAME_IN(self)
    %  char = NAME_IN(self, int ind)
    %
    %Get input scheme name by index.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vi
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L320
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L974-L980
    %
    %
    %
    %.......
    %
    %::
    %
    %  NAME_IN(self, int ind)
    %
    %
    %
    %[INTERNAL] 
    %Get input scheme name by index.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vi
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L320
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L974-L980
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  NAME_IN(self)
    %
    %
    %
    %[INTERNAL] 
    %Get input scheme.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vg
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L310
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L950-L952
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(707, self, varargin{:});
    end
    function varargout = name_out(self,varargin)
    %NAME_OUT [INTERNAL] 
    %
    %  {char} = NAME_OUT(self)
    %  char = NAME_OUT(self, int ind)
    %
    %Get output scheme name by index.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vj
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L325
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L982-L988
    %
    %
    %
    %.......
    %
    %::
    %
    %  NAME_OUT(self, int ind)
    %
    %
    %
    %[INTERNAL] 
    %Get output scheme name by index.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vj
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L325
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L982-L988
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  NAME_OUT(self)
    %
    %
    %
    %[INTERNAL] 
    %Get output scheme.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vh
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L315
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L954-L956
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(708, self, varargin{:});
    end
    function varargout = index_in(self,varargin)
    %INDEX_IN [INTERNAL] 
    %
    %  int = INDEX_IN(self, char name)
    %
    %Find the index for a string describing a particular entry of an 
    %input 
    %scheme.
    %
    %example: schemeEntry("x_opt") -> returns NLPSOL_X if 
    %FunctionInternal 
    %adheres to SCHEME_NLPINput
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vk
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L333
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L958-L964
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(709, self, varargin{:});
    end
    function varargout = index_out(self,varargin)
    %INDEX_OUT [INTERNAL] 
    %
    %  int = INDEX_OUT(self, char name)
    %
    %Find the index for a string describing a particular entry of an 
    %output
    % scheme.
    %
    %example: schemeEntry("x_opt") -> returns NLPSOL_X if 
    %FunctionInternal 
    %adheres to SCHEME_NLPINput
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vl
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L341
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L966-L972
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(710, self, varargin{:});
    end
    function varargout = default_in(self,varargin)
    %DEFAULT_IN [INTERNAL] 
    %
    %  double = DEFAULT_IN(self, int ind)
    %
    %Get default input value.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vm
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L346
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1454-L1456
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(711, self, varargin{:});
    end
    function varargout = max_in(self,varargin)
    %MAX_IN [INTERNAL] 
    %
    %  double = MAX_IN(self, int ind)
    %
    %Get largest input value.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vn
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L351
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1458-L1460
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(712, self, varargin{:});
    end
    function varargout = min_in(self,varargin)
    %MIN_IN [INTERNAL] 
    %
    %  double = MIN_IN(self, int ind)
    %
    %Get smallest input value.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vo
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L356
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1462-L1464
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(713, self, varargin{:});
    end
    function varargout = nominal_in(self,varargin)
    %NOMINAL_IN [INTERNAL] 
    %
    %  [double] = NOMINAL_IN(self, int ind)
    %
    %Get nominal input value.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vp
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L361
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1466-L1468
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(714, self, varargin{:});
    end
    function varargout = nominal_out(self,varargin)
    %NOMINAL_OUT [INTERNAL] 
    %
    %  [double] = NOMINAL_OUT(self, int ind)
    %
    %Get nominal output value.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vq
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L366
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1470-L1472
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(715, self, varargin{:});
    end
    function varargout = sparsity_in(self,varargin)
    %SPARSITY_IN [INTERNAL] 
    %
    %  Sparsity = SPARSITY_IN(self, int ind)
    %  Sparsity = SPARSITY_IN(self, char iname)
    %
    %Get sparsity of a given input.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vr
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L373
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L998-L1004
    %
    %
    %
    %.......
    %
    %::
    %
    %  SPARSITY_IN(self, int ind)
    %
    %
    %
    %[INTERNAL] 
    %Get sparsity of a given input.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vr
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L372
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L990-L996
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  SPARSITY_IN(self, char iname)
    %
    %
    %
    %[INTERNAL] 
    %Get sparsity of a given input.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vr
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L373
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L998-L1004
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(716, self, varargin{:});
    end
    function varargout = sparsity_out(self,varargin)
    %SPARSITY_OUT [INTERNAL] 
    %
    %  Sparsity = SPARSITY_OUT(self, int ind)
    %  Sparsity = SPARSITY_OUT(self, char iname)
    %
    %Get sparsity of a given output.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vs
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L381
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1014-L1020
    %
    %
    %
    %.......
    %
    %::
    %
    %  SPARSITY_OUT(self, int ind)
    %
    %
    %
    %[INTERNAL] 
    %Get sparsity of a given output.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vs
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L380
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1006-L1012
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  SPARSITY_OUT(self, char iname)
    %
    %
    %
    %[INTERNAL] 
    %Get sparsity of a given output.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vs
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L381
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1014-L1020
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(717, self, varargin{:});
    end
    function varargout = is_diff_in(self,varargin)
    %IS_DIFF_IN [INTERNAL] 
    %
    %  [bool] = IS_DIFF_IN(self)
    %  bool = IS_DIFF_IN(self, int ind)
    %
    %Get differentiability of inputs/output.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vt
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L390
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1038-L1044
    %
    %
    %
    %.......
    %
    %::
    %
    %  IS_DIFF_IN(self)
    %
    %
    %
    %[INTERNAL] 
    %Get differentiability of inputs/output.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vt
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L390
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1038-L1044
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  IS_DIFF_IN(self, int ind)
    %
    %
    %
    %[INTERNAL] 
    %Get differentiability of inputs/output.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vt
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L388
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1022-L1028
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(718, self, varargin{:});
    end
    function varargout = is_diff_out(self,varargin)
    %IS_DIFF_OUT [INTERNAL] 
    %
    %  [bool] = IS_DIFF_OUT(self)
    %  bool = IS_DIFF_OUT(self, int ind)
    %
    %Get differentiability of inputs/output.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vt
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L391
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1046-L1052
    %
    %
    %
    %.......
    %
    %::
    %
    %  IS_DIFF_OUT(self)
    %
    %
    %
    %[INTERNAL] 
    %Get differentiability of inputs/output.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vt
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L391
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1046-L1052
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  IS_DIFF_OUT(self, int ind)
    %
    %
    %
    %[INTERNAL] 
    %Get differentiability of inputs/output.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vt
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L389
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1030-L1036
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(719, self, varargin{:});
    end
    function varargout = factory(self,varargin)
    %FACTORY [INTERNAL] 
    %
    %  Function = FACTORY(self, char name, {char} s_in, {char} s_out, struct:{char} aux, struct opts)
    %
    %
      [varargout{1:nargout}] = casadiMEX(720, self, varargin{:});
    end
    function varargout = oracle(self,varargin)
    %ORACLE [INTERNAL] 
    %
    %  Function = ORACLE(self)
    %
    %Get oracle.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vu
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L407
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1877-L1883
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(721, self, varargin{:});
    end
    function varargout = wrap(self,varargin)
    %WRAP [INTERNAL] 
    %
    %  Function = WRAP(self)
    %
    %Wrap in an  Function instance consisting of only one  MX call.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vv
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L412
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1885-L1887
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(722, self, varargin{:});
    end
    function varargout = wrap_as_needed(self,varargin)
    %WRAP_AS_NEEDED [INTERNAL] 
    %
    %  Function = WRAP_AS_NEEDED(self, struct opts)
    %
    %Wrap in a  Function with options.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vw
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L417
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1889-L1891
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(723, self, varargin{:});
    end
    function varargout = which_depends(self,varargin)
    %WHICH_DEPENDS [INTERNAL] 
    %
    %  [bool] = WHICH_DEPENDS(self, char s_in, {char} s_out, int order, bool tr)
    %
    %Which variables enter with some order.
    %
    %Parameters:
    %-----------
    %
    %order: 
    %Only 1 (linear) and 2 (nonlinear) allowed
    %
    %tr: 
    %Flip the relationship. Return which expressions contain the variables
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vx
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L425
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1795-L1802
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(724, self, varargin{:});
    end
    function varargout = print_dimensions(self,varargin)
    %PRINT_DIMENSIONS [INTERNAL] 
    %
    %  std::ostream & = PRINT_DIMENSIONS(self)
    %
    %Print dimensions of inputs and outputs.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vy
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L432
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1126-L1128
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(725, self, varargin{:});
    end
    function varargout = print_options(self,varargin)
    %PRINT_OPTIONS [INTERNAL] 
    %
    %  std::ostream & = PRINT_OPTIONS(self)
    %
    %Print options to a stream.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1vz
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L437
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1130-L1132
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(726, self, varargin{:});
    end
    function varargout = print_option(self,varargin)
    %PRINT_OPTION [INTERNAL] 
    %
    %  std::ostream & = PRINT_OPTION(self, char name)
    %
    %Print all information there is to know about a certain option.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1w0
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L442
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1134-L1136
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(727, self, varargin{:});
    end
    function varargout = has_option(self,varargin)
    %HAS_OPTION [INTERNAL] 
    %
    %  bool = HAS_OPTION(self, char option_name)
    %
    %Does a particular option exist.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1w1
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L447
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1138-L1145
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(728, self, varargin{:});
    end
    function varargout = change_option(self,varargin)
    %CHANGE_OPTION [INTERNAL] 
    %
    %  CHANGE_OPTION(self, char option_name, GenericType option_value)
    %
    %Change option after object creation for debugging.
    %
    %This is only possible for a selected number of options that do not 
    %change 
    %the numerical results of the computation, e.g. to enable a more
    % verbose 
    %output or saving to file.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1w2
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L455
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1147-L1157
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(729, self, varargin{:});
    end
    function varargout = uses_output(self,varargin)
    %USES_OUTPUT [INTERNAL] 
    %
    %  bool = USES_OUTPUT(self)
    %
    %Do the derivative functions need nondifferentiated outputs?
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1w3
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L460
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L882-L884
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(730, self, varargin{:});
    end
    function varargout = jacobian_old(self,varargin)
    %JACOBIAN_OLD [DEPRECATED] Replaced by  Function::factory.
    %
    %  Function = JACOBIAN_OLD(self, int iind, int oind)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1w4
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L466
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L887-L893
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(731, self, varargin{:});
    end
    function varargout = hessian_old(self,varargin)
    %HESSIAN_OLD [DEPRECATED] Replaced by  Function::factory.
    %
    %  Function = HESSIAN_OLD(self, int iind, int oind)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1w5
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L471
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L895-L903
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(732, self, varargin{:});
    end
    function varargout = sparsity_jac(self,varargin)
    %SPARSITY_JAC [DEPRECATED] Get, if necessary generate, the sparsity of a Jacobian 
    %
    %  Sparsity = SPARSITY_JAC(self, char iind, int oind, bool compact, bool symmetric)
    %  Sparsity = SPARSITY_JAC(self, int iind, int oind, bool compact, bool symmetric)
    %  Sparsity = SPARSITY_JAC(self, int iind, char oind, bool compact, bool symmetric)
    %  Sparsity = SPARSITY_JAC(self, char iind, char oind, bool compact, bool symmetric)
    %
    %block
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L485
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L485-L488
    %
    %
    %
    %.......
    %
    %::
    %
    %  SPARSITY_JAC(self, char iind, int oind, bool compact, bool symmetric)
    %
    %
    %
    %[DEPRECATED] Get, if necessary generate, the sparsity of a Jacobian 
    %block
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L477
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L477-L480
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  SPARSITY_JAC(self, int iind, int oind, bool compact, bool symmetric)
    %
    %
    %
    %[DEPRECATED] Get, if necessary generate, the sparsity of a Jacobian 
    %block
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L475
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L906-L912
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  SPARSITY_JAC(self, int iind, char oind, bool compact, bool symmetric)
    %
    %
    %
    %[DEPRECATED] Get, if necessary generate, the sparsity of a Jacobian 
    %block
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L481
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L481-L484
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  SPARSITY_JAC(self, char iind, char oind, bool compact, bool symmetric)
    %
    %
    %
    %[DEPRECATED] Get, if necessary generate, the sparsity of a Jacobian 
    %block
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L485
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L485-L488
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(733, self, varargin{:});
    end
    function varargout = jacobian(self,varargin)
    %JACOBIAN [INTERNAL] 
    %
    %  Function = JACOBIAN(self)
    %
    %Calculate all Jacobian blocks.
    %
    %Generates a function that takes all non-differentiated inputs and 
    %outputs 
    %and calculates all Jacobian blocks. Inputs that are not needed
    % by the 
    %routine are all-zero sparse matrices with the correct 
    %dimensions.  Output 
    %blocks that are not calculated, e.g. if the corresponding input or 
    %output 
    %is marked non-differentiated are also all-zero sparse. The 
    %Jacobian blocks 
    %are sorted starting by all the blocks for the first 
    %output, then all the 
    %blocks for the second output and so on. E.g. f : 
    %(x, y) -> (r, s) results 
    %in the function jac_f : (x, y, out_r, out_s) 
    %-> (jac_r_x, jac_r_y, jac_s_x,
    % jac_s_y)
    %
    %This function is cached.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1w6
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L508
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L915-L921
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(734, self, varargin{:});
    end
    function varargout = call(self,varargin)
    %CALL [INTERNAL] 
    %
    %  struct:DM = CALL(self, struct:DM arg, bool always_inline, bool never_inline)
    %  {DM} = CALL(self, {DM} arg, bool always_inline, bool never_inline)
    %  {SX} = CALL(self, {SX} arg, bool always_inline, bool never_inline)
    %  struct:SX = CALL(self, struct:SX arg, bool always_inline, bool never_inline)
    %  struct:MX = CALL(self, struct:MX arg, bool always_inline, bool never_inline)
    %  {MX} = CALL(self, {MX} arg, bool always_inline, bool never_inline)
    %
    %Evaluate the function symbolically or numerically.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1w7
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L524
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1445-L1452
    %
    %
    %
    %.......
    %
    %::
    %
    %  CALL(self, struct:DM arg, bool always_inline, bool never_inline)
    %
    %
    %
    %[INTERNAL] 
    %Evaluate the function symbolically or numerically.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1w7
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L520
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1427-L1434
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  CALL(self, {DM} arg, bool always_inline, bool never_inline)
    %
    %
    %
    %[INTERNAL] 
    %Evaluate the function symbolically or numerically.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1w7
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L514
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L356-L363
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  CALL(self, {SX} arg, bool always_inline, bool never_inline)
    %
    %
    %
    %[INTERNAL] 
    %Evaluate the function symbolically or numerically.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1w7
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L516
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L365-L372
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  CALL(self, struct:SX arg, bool always_inline, bool never_inline)
    %
    %
    %
    %[INTERNAL] 
    %Evaluate the function symbolically or numerically.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1w7
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L522
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1436-L1443
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  CALL(self, struct:MX arg, bool always_inline, bool never_inline)
    %
    %
    %
    %[INTERNAL] 
    %Evaluate the function symbolically or numerically.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1w7
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L524
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1445-L1452
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  CALL(self, {MX} arg, bool always_inline, bool never_inline)
    %
    %
    %
    %[INTERNAL] 
    %Evaluate the function symbolically or numerically.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1w7
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L518
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L374-L381
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(735, self, varargin{:});
    end
    function varargout = mapsum(self,varargin)
    %MAPSUM [INTERNAL] 
    %
    %  {MX} = MAPSUM(self, {MX} x, char parallelization)
    %
    %Evaluate symbolically in parallel and sum (matrix graph)
    %
    %Parameters:
    %-----------
    %
    %parallelization: 
    %Type of parallelization used: unroll|serial|openmp
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1wh
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L642
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L755-L762
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(736, self, varargin{:});
    end
    function varargout = mapaccum(self,varargin)
    %MAPACCUM [INTERNAL] 
    %
    %  Function = MAPACCUM(self, int N, struct opts)
    %  Function = MAPACCUM(self, char name, int N, struct opts)
    %  Function = MAPACCUM(self, char name, int N, int n_accum, struct opts)
    %  Function = MAPACCUM(self, char name, int n, {char} accum_in, {char} accum_out, struct opts)
    %  Function = MAPACCUM(self, char name, int n, [int] accum_in, [int] accum_out, struct opts)
    %
    %Create a mapaccumulated version of this function.
    %
    %Suppose the function has a signature of:
    %
    %::
    %
    %     f: (x, u) -> (x_next , y )
    %  
    %
    %
    %
    %The the mapaccumulated version has the signature:
    %
    %::
    %
    %     F: (x0, U) -> (X , Y )
    %  
    %      with
    %          U: horzcat([u0, u1, ..., u_(N-1)])
    %          X: horzcat([x1, x2, ..., x_N])
    %          Y: horzcat([y0, y1, ..., y_(N-1)])
    %  
    %      and
    %          x1, y0 <- f(x0, u0)
    %          x2, y1 <- f(x1, u1)
    %          ...
    %          x_N, y_(N-1) <- f(x_(N-1), u_(N-1))
    %  
    %
    %
    %
    %Mapaccum has the following benefits over writing an equivalent for-
    %loop:
    %
    %much faster at construction time
    %
    %potentially much faster compilation times (for codegen)
    %
    %offers a trade-off between memory and evaluation time
    %
    %The base (settable through the options dictionary, default 10), is 
    %used to 
    %create a tower of function calls, containing unrolled for-
    %loops of length 
    %maximum base.
    %
    %This technique is much more scalable in terms of memory-usage, but 
    %slightly
    % slower at evaluation, than a plain for-loop. The effect is 
    %similar to that
    % of a for-loop with a check-pointing instruction after 
    %each chunk of 
    %iterations with size base.
    %
    %Set base to -1 to unroll all the way; no gains in memory efficiency 
    %here.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1wi
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L697
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L515-L517
    %
    %
    %
    %.......
    %
    %::
    %
    %  MAPACCUM(self, int N, struct opts)
    %
    %
    %
    %[INTERNAL] 
    %Create a mapaccumulated version of this function.
    %
    %Suppose the function has a signature of:
    %
    %::
    %
    %     f: (x, u) -> (x_next , y )
    %  
    %
    %
    %
    %The the mapaccumulated version has the signature:
    %
    %::
    %
    %     F: (x0, U) -> (X , Y )
    %  
    %      with
    %          U: horzcat([u0, u1, ..., u_(N-1)])
    %          X: horzcat([x1, x2, ..., x_N])
    %          Y: horzcat([y0, y1, ..., y_(N-1)])
    %  
    %      and
    %          x1, y0 <- f(x0, u0)
    %          x2, y1 <- f(x1, u1)
    %          ...
    %          x_N, y_(N-1) <- f(x_(N-1), u_(N-1))
    %  
    %
    %
    %
    %Mapaccum has the following benefits over writing an equivalent for-
    %loop:
    %
    %much faster at construction time
    %
    %potentially much faster compilation times (for codegen)
    %
    %offers a trade-off between memory and evaluation time
    %
    %The base (settable through the options dictionary, default 10), is 
    %used to 
    %create a tower of function calls, containing unrolled for-
    %loops of length 
    %maximum base.
    %
    %This technique is much more scalable in terms of memory-usage, but 
    %slightly
    % slower at evaluation, than a plain for-loop. The effect is 
    %similar to that
    % of a for-loop with a check-pointing instruction after 
    %each chunk of 
    %iterations with size base.
    %
    %Set base to -1 to unroll all the way; no gains in memory efficiency 
    %here.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1wi
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L697
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L515-L517
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  MAPACCUM(self, char name, int N, struct opts)
    %
    %
    %
    %[INTERNAL] 
    %Create a mapaccumulated version of this function.
    %
    %Suppose the function has a signature of:
    %
    %::
    %
    %     f: (x, u) -> (x_next , y )
    %  
    %
    %
    %
    %The the mapaccumulated version has the signature:
    %
    %::
    %
    %     F: (x0, U) -> (X , Y )
    %  
    %      with
    %          U: horzcat([u0, u1, ..., u_(N-1)])
    %          X: horzcat([x1, x2, ..., x_N])
    %          Y: horzcat([y0, y1, ..., y_(N-1)])
    %  
    %      and
    %          x1, y0 <- f(x0, u0)
    %          x2, y1 <- f(x1, u1)
    %          ...
    %          x_N, y_(N-1) <- f(x_(N-1), u_(N-1))
    %  
    %
    %
    %
    %Mapaccum has the following benefits over writing an equivalent for-
    %loop:
    %
    %much faster at construction time
    %
    %potentially much faster compilation times (for codegen)
    %
    %offers a trade-off between memory and evaluation time
    %
    %The base (settable through the options dictionary, default 10), is 
    %used to 
    %create a tower of function calls, containing unrolled for-
    %loops of length 
    %maximum base.
    %
    %This technique is much more scalable in terms of memory-usage, but 
    %slightly
    % slower at evaluation, than a plain for-loop. The effect is 
    %similar to that
    % of a for-loop with a check-pointing instruction after 
    %each chunk of 
    %iterations with size base.
    %
    %Set base to -1 to unroll all the way; no gains in memory efficiency 
    %here.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1wi
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L686
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L518-L520
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  MAPACCUM(self, char name, int N, int n_accum, struct opts)
    %
    %
    %
    %[INTERNAL] 
    %Create a mapaccumulated version of this function.
    %
    %Suppose the function has a signature of:
    %
    %::
    %
    %     f: (x, u) -> (x_next , y )
    %  
    %
    %
    %
    %The the mapaccumulated version has the signature:
    %
    %::
    %
    %     F: (x0, U) -> (X , Y )
    %  
    %      with
    %          U: horzcat([u0, u1, ..., u_(N-1)])
    %          X: horzcat([x1, x2, ..., x_N])
    %          Y: horzcat([y0, y1, ..., y_(N-1)])
    %  
    %      and
    %          x1, y0 <- f(x0, u0)
    %          x2, y1 <- f(x1, u1)
    %          ...
    %          x_N, y_(N-1) <- f(x_(N-1), u_(N-1))
    %  
    %
    %
    %
    %Mapaccum has the following benefits over writing an equivalent for-
    %loop:
    %
    %much faster at construction time
    %
    %potentially much faster compilation times (for codegen)
    %
    %offers a trade-off between memory and evaluation time
    %
    %The base (settable through the options dictionary, default 10), is 
    %used to 
    %create a tower of function calls, containing unrolled for-
    %loops of length 
    %maximum base.
    %
    %This technique is much more scalable in terms of memory-usage, but 
    %slightly
    % slower at evaluation, than a plain for-loop. The effect is 
    %similar to that
    % of a for-loop with a check-pointing instruction after 
    %each chunk of 
    %iterations with size base.
    %
    %Set base to -1 to unroll all the way; no gains in memory efficiency 
    %here.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1wi
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L687
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L521-L549
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  MAPACCUM(self, char name, int n, {char} accum_in, {char} accum_out, struct opts)
    %
    %
    %
    %[INTERNAL] 
    %Create a mapaccumulated version of this function.
    %
    %Suppose the function has a signature of:
    %
    %::
    %
    %     f: (x, u) -> (x_next , y )
    %  
    %
    %
    %
    %The the mapaccumulated version has the signature:
    %
    %::
    %
    %     F: (x0, U) -> (X , Y )
    %  
    %      with
    %          U: horzcat([u0, u1, ..., u_(N-1)])
    %          X: horzcat([x1, x2, ..., x_N])
    %          Y: horzcat([y0, y1, ..., y_(N-1)])
    %  
    %      and
    %          x1, y0 <- f(x0, u0)
    %          x2, y1 <- f(x1, u1)
    %          ...
    %          x_N, y_(N-1) <- f(x_(N-1), u_(N-1))
    %  
    %
    %
    %
    %Mapaccum has the following benefits over writing an equivalent for-
    %loop:
    %
    %much faster at construction time
    %
    %potentially much faster compilation times (for codegen)
    %
    %offers a trade-off between memory and evaluation time
    %
    %The base (settable through the options dictionary, default 10), is 
    %used to 
    %create a tower of function calls, containing unrolled for-
    %loops of length 
    %maximum base.
    %
    %This technique is much more scalable in terms of memory-usage, but 
    %slightly
    % slower at evaluation, than a plain for-loop. The effect is 
    %similar to that
    % of a for-loop with a check-pointing instruction after 
    %each chunk of 
    %iterations with size base.
    %
    %Set base to -1 to unroll all the way; no gains in memory efficiency 
    %here.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1wi
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L693
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L626-L634
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  MAPACCUM(self, char name, int n, [int] accum_in, [int] accum_out, struct opts)
    %
    %
    %
    %[INTERNAL] 
    %Create a mapaccumulated version of this function.
    %
    %Suppose the function has a signature of:
    %
    %::
    %
    %     f: (x, u) -> (x_next , y )
    %  
    %
    %
    %
    %The the mapaccumulated version has the signature:
    %
    %::
    %
    %     F: (x0, U) -> (X , Y )
    %  
    %      with
    %          U: horzcat([u0, u1, ..., u_(N-1)])
    %          X: horzcat([x1, x2, ..., x_N])
    %          Y: horzcat([y0, y1, ..., y_(N-1)])
    %  
    %      and
    %          x1, y0 <- f(x0, u0)
    %          x2, y1 <- f(x1, u1)
    %          ...
    %          x_N, y_(N-1) <- f(x_(N-1), u_(N-1))
    %  
    %
    %
    %
    %Mapaccum has the following benefits over writing an equivalent for-
    %loop:
    %
    %much faster at construction time
    %
    %potentially much faster compilation times (for codegen)
    %
    %offers a trade-off between memory and evaluation time
    %
    %The base (settable through the options dictionary, default 10), is 
    %used to 
    %create a tower of function calls, containing unrolled for-
    %loops of length 
    %maximum base.
    %
    %This technique is much more scalable in terms of memory-usage, but 
    %slightly
    % slower at evaluation, than a plain for-loop. The effect is 
    %similar to that
    % of a for-loop with a check-pointing instruction after 
    %each chunk of 
    %iterations with size base.
    %
    %Set base to -1 to unroll all the way; no gains in memory efficiency 
    %here.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1wi
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L689
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L596-L624
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(737, self, varargin{:});
    end
    function varargout = fold(self,varargin)
    %FOLD [INTERNAL] 
    %
    %  Function = FOLD(self, int N, struct opts)
    %
    %Create a mapaccumulated version of this function.
    %
    %Suppose the function has a signature of:
    %
    %::
    %
    %     f: (x, u) -> (x_next , y )
    %  
    %
    %
    %
    %The the mapaccumulated version has the signature:
    %
    %::
    %
    %     F: (x0, U) -> (X , Y )
    %  
    %      with
    %          U: horzcat([u0, u1, ..., u_(N-1)])
    %          X: horzcat([x1, x2, ..., x_N])
    %          Y: horzcat([y0, y1, ..., y_(N-1)])
    %  
    %      and
    %          x1, y0 <- f(x0, u0)
    %          x2, y1 <- f(x1, u1)
    %          ...
    %          x_N, y_(N-1) <- f(x_(N-1), u_(N-1))
    %  
    %
    %
    %
    %Mapaccum has the following benefits over writing an equivalent for-
    %loop:
    %
    %much faster at construction time
    %
    %potentially much faster compilation times (for codegen)
    %
    %offers a trade-off between memory and evaluation time
    %
    %The base (settable through the options dictionary, default 10), is 
    %used to 
    %create a tower of function calls, containing unrolled for-
    %loops of length 
    %maximum base.
    %
    %This technique is much more scalable in terms of memory-usage, but 
    %slightly
    % slower at evaluation, than a plain for-loop. The effect is 
    %similar to that
    % of a for-loop with a check-pointing instruction after 
    %each chunk of 
    %iterations with size base.
    %
    %Set base to -1 to unroll all the way; no gains in memory efficiency 
    %here.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1wi
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L698
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L508-L514
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(738, self, varargin{:});
    end
    function varargout = map(self,varargin)
    %MAP [INTERNAL] 
    %
    %  Function = MAP(self, int n, char parallelization)
    %  Function = MAP(self, int n, [bool] reduce_in, [bool] reduce_out, struct opts)
    %  Function = MAP(self, int n, char parallelization, int max_num_threads)
    %  Function = MAP(self, char name, char parallelization, int n, {char} reduce_in, {char} reduce_out, struct opts)
    %  Function = MAP(self, char name, char parallelization, int n, [int] reduce_in, [int] reduce_out, struct opts)
    %
    %Map with reduction.
    %
    %A subset of the inputs are non-repeated and a subset of the outputs 
    %summed 
    %up.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1wk
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L745
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L636-L642
    %
    %
    %
    %.......
    %
    %::
    %
    %  MAP(self, int n, [bool] reduce_in, [bool] reduce_out, struct opts)
    %
    %
    %
    %[INTERNAL] 
    %Map with reduction.
    %
    %A subset of the inputs are non-repeated and a subset of the outputs 
    %summed 
    %up.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1wk
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L745
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L636-L642
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  MAP(self, char name, char parallelization, int n, {char} reduce_in, {char} reduce_out, struct opts)
    %
    %
    %
    %[INTERNAL] 
    %Map with reduction.
    %
    %A subset of the inputs are non-repeated and a subset of the outputs 
    %summed 
    %up.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1wk
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L741
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L667-L674
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  MAP(self, char name, char parallelization, int n, [int] reduce_in, [int] reduce_out, struct opts)
    %
    %
    %
    %[INTERNAL] 
    %Map with reduction.
    %
    %A subset of the inputs are non-repeated and a subset of the outputs 
    %summed 
    %up.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1wk
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L737
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L644-L665
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  MAP(self, int n, char parallelization, int max_num_threads)
    %
    %
    %
    %[INTERNAL] 
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  MAP(self, int n, char parallelization)
    %
    %
    %
    %[INTERNAL] 
    %Create a mapped version of this function.
    %
    %Suppose the function has a signature of:
    %
    %::
    %
    %     f: (a, p) -> ( s )
    %  
    %
    %
    %
    %The the mapped version has the signature:
    %
    %::
    %
    %     F: (A, P) -> (S )
    %  
    %      with
    %          A: horzcat([a0, a1, ..., a_(N-1)])
    %          P: horzcat([p0, p1, ..., p_(N-1)])
    %          S: horzcat([s0, s1, ..., s_(N-1)])
    %      and
    %          s0 <- f(a0, p0)
    %          s1 <- f(a1, p1)
    %          ...
    %          s_(N-1) <- f(a_(N-1), p_(N-1))
    %  
    %
    %
    %
    %Parameters:
    %-----------
    %
    %parallelization: 
    %Type of parallelization used: unroll|serial|openmp
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1wj
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L726
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L708-L743
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(739, self, varargin{:});
    end
    function varargout = slice(self,varargin)
    %SLICE [INTERNAL] 
    %
    %  Function = SLICE(self, char name, [int] order_in, [int] order_out, struct opts)
    %
    %returns a new function with a selection of inputs/outputs of the
    % 
    %original
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1wl
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L754
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L746-L753
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(740, self, varargin{:});
    end
    function varargout = forward(self,varargin)
    %FORWARD [INTERNAL] 
    %
    %  Function = FORWARD(self, int nfwd)
    %
    %Get a function that calculates  nfwd forward derivatives.
    %
    %
    %
    %::
    %
    %     Returns a function with <tt>n_in + n_out + n_in</tt> inputs
    %     and <tt>nfwd</tt> outputs.
    %     The first <tt>n_in</tt> inputs correspond to nondifferentiated inputs.
    %     The next <tt>n_out</tt> inputs correspond to nondifferentiated outputs.
    %     and the last <tt>n_in</tt> inputs correspond to forward seeds,
    %     stacked horizontally
    %     The  <tt>n_out</tt> outputs correspond to forward sensitivities,
    %     stacked horizontally.     *
    %     <tt>(n_in = n_in(), n_out = n_out())</tt>
    %  
    %    The functions returned are cached, meaning that if called multiple timed
    %    with the same value, then multiple references to the same function will be returned.
    %  
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1wq
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L800
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1110-L1116
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(744, self, varargin{:});
    end
    function varargout = reverse(self,varargin)
    %REVERSE [INTERNAL] 
    %
    %  Function = REVERSE(self, int nadj)
    %
    %Get a function that calculates  nadj adjoint derivatives.
    %
    %
    %
    %::
    %
    %     Returns a function with <tt>n_in + n_out + n_out</tt> inputs
    %     and <tt>n_in</tt> outputs.
    %     The first <tt>n_in</tt> inputs correspond to nondifferentiated inputs.
    %     The next <tt>n_out</tt> inputs correspond to nondifferentiated outputs.
    %     and the last <tt>n_out</tt> inputs correspond to adjoint seeds,
    %     stacked horizontally
    %     The  <tt>n_in</tt> outputs correspond to adjoint sensitivities,
    %     stacked horizontally.     *
    %     <tt>(n_in = n_in(), n_out = n_out())</tt>
    %  
    %     <tt>(n_in = n_in(), n_out = n_out())</tt>
    %  
    %    The functions returned are cached, meaning that if called multiple timed
    %    with the same value, then multiple references to the same function will be returned.
    %  
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1wr
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L820
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1118-L1124
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(745, self, varargin{:});
    end
    function varargout = jac_sparsity(self,varargin)
    %JAC_SPARSITY [INTERNAL] 
    %
    %  {Sparsity} = JAC_SPARSITY(self, bool compact)
    %  Sparsity = JAC_SPARSITY(self, int oind, int iind, bool compact)
    %
    %Get, if necessary generate, the sparsity of a single Jacobian 
    %block.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1wt
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L830
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L942-L948
    %
    %
    %
    %.......
    %
    %::
    %
    %  JAC_SPARSITY(self, bool compact)
    %
    %
    %
    %[INTERNAL] 
    %Get, if necessary generate, the sparsity of all Jacobian blocks.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1ws
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L825
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L931-L940
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  JAC_SPARSITY(self, int oind, int iind, bool compact)
    %
    %
    %
    %[INTERNAL] 
    %Get, if necessary generate, the sparsity of a single Jacobian 
    %block.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1wt
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L830
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L942-L948
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(746, self, varargin{:});
    end
    function varargout = generate(self,varargin)
    %GENERATE [INTERNAL] 
    %
    %  char = GENERATE(self, struct opts)
    %  char = GENERATE(self, char fname, struct opts)
    %
    %Export / Generate C code for the function.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1wv
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L840
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1163-L1165
    %
    %
    %
    %.......
    %
    %::
    %
    %  GENERATE(self, struct opts)
    %
    %
    %
    %[INTERNAL] 
    %Export / Generate C code for the function.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1wv
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L840
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1163-L1165
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  GENERATE(self, char fname, struct opts)
    %
    %
    %
    %[INTERNAL] 
    %Export / Generate C code for the function.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1wu
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L835
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1167-L1171
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(747, self, varargin{:});
    end
    function varargout = generate_dependencies(self,varargin)
    %GENERATE_DEPENDENCIES [INTERNAL] 
    %
    %  char = GENERATE_DEPENDENCIES(self, char fname, struct opts)
    %
    %Export / Generate C code for the dependency function.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1ww
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L845
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1173-L1175
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(748, self, varargin{:});
    end
    function varargout = generate_in(self,varargin)
    %GENERATE_IN [INTERNAL] 
    %
    %  {DM} = GENERATE_IN(self, char fname)
    %  GENERATE_IN(self, char fname, {DM} arg)
    %
    %Export an input file that can be passed to generate C code with 
    %a 
    %main.
    %
    %See: 
    % generate_out
    %
    %See: 
    % convert_in to convert between dict/map and vector
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1wx
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L855
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1207-L1217
    %
    %
    %
    %.......
    %
    %::
    %
    %  GENERATE_IN(self, char fname)
    %
    %
    %
    %[INTERNAL] 
    %Export an input file that can be passed to generate C code with 
    %a 
    %main.
    %
    %See: 
    % generate_out
    %
    %See: 
    % convert_in to convert between dict/map and vector
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1wx
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L855
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1207-L1217
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  GENERATE_IN(self, char fname, {DM} arg)
    %
    %
    %
    %[INTERNAL] 
    %Export an input file that can be passed to generate C code with 
    %a 
    %main.
    %
    %See: 
    % generate_out
    %
    %See: 
    % convert_in to convert between dict/map and vector
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1wx
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L854
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1177-L1190
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(749, self, varargin{:});
    end
    function varargout = generate_out(self,varargin)
    %GENERATE_OUT [INTERNAL] 
    %
    %  {DM} = GENERATE_OUT(self, char fname)
    %  GENERATE_OUT(self, char fname, {DM} arg)
    %
    %Export an output file that can be checked with generated C code
    % 
    %output.
    %
    %See: 
    % generate_in
    %
    %See: 
    % convert_out to convert between dict/map and vector
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1wy
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L866
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1219-L1229
    %
    %
    %
    %.......
    %
    %::
    %
    %  GENERATE_OUT(self, char fname)
    %
    %
    %
    %[INTERNAL] 
    %Export an output file that can be checked with generated C code
    % 
    %output.
    %
    %See: 
    % generate_in
    %
    %See: 
    % convert_out to convert between dict/map and vector
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1wy
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L866
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1219-L1229
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  GENERATE_OUT(self, char fname, {DM} arg)
    %
    %
    %
    %[INTERNAL] 
    %Export an output file that can be checked with generated C code
    % 
    %output.
    %
    %See: 
    % generate_in
    %
    %See: 
    % convert_out to convert between dict/map and vector
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1wy
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L865
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1192-L1205
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(750, self, varargin{:});
    end
    function varargout = serialize(self,varargin)
    %SERIALIZE [INTERNAL] 
    %
    %  char = SERIALIZE(self, struct opts)
    %
    %Serialize.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1x2
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L893
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1248-L1252
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(751, self, varargin{:});
    end
    function varargout = save(self,varargin)
    %SAVE [INTERNAL] 
    %
    %  SAVE(self, char fname, struct opts)
    %
    %Save  Function to a file.
    %
    %See: 
    % load
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_240
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L900
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1243-L1246
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(752, self, varargin{:});
    end
    function varargout = export_code(self,varargin)
    %EXPORT_CODE [INTERNAL] 
    %
    %  char = EXPORT_CODE(self, char lang, struct options)
    %  EXPORT_CODE(self, char lang, char fname, struct options)
    %
    %Export function in specific language.
    %
    %Only allowed for (a subset of) SX/MX Functions
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1wz
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L902
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1275-L1279
    %
    %
    %
    %.......
    %
    %::
    %
    %  EXPORT_CODE(self, char lang, struct options)
    %
    %
    %
    %[INTERNAL] 
    %Export function in specific language.
    %
    %Only allowed for (a subset of) SX/MX Functions
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1wz
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L902
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1275-L1279
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  EXPORT_CODE(self, char lang, char fname, struct options)
    %
    %
    %
    %[INTERNAL] 
    %Export function in specific language.
    %
    %Only allowed for (a subset of) SX/MX Functions
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1wz
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L875
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1236-L1240
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(753, self, varargin{:});
    end
    function varargout = stats(self,varargin)
    %STATS [INTERNAL] 
    %
    %  struct = STATS(self, int mem)
    %
    %Get all statistics obtained at the end of the last evaluate 
    %call.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L932
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L927-L929
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(754, self, varargin{:});
    end
    function varargout = sx_in(self,varargin)
    %SX_IN [INTERNAL] 
    %
    %  {SX} = SX_IN(self)
    %  SX = SX_IN(self, int iind)
    %  SX = SX_IN(self, char iname)
    %
    %Get symbolic primitives equivalent to the input expressions.
    %
    %There is no guarantee that subsequent calls return unique answers
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1x4
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L944
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1542-L1548
    %
    %
    %
    %.......
    %
    %::
    %
    %  SX_IN(self)
    %
    %
    %
    %[INTERNAL] 
    %Get symbolic primitives equivalent to the input expressions.
    %
    %There is no guarantee that subsequent calls return unique answers
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1x4
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L944
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1542-L1548
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  SX_IN(self, int iind)
    %
    %
    %
    %[INTERNAL] 
    %Get symbolic primitives equivalent to the input expressions.
    %
    %There is no guarantee that subsequent calls return unique answers
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1x4
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L940
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1526-L1532
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  SX_IN(self, char iname)
    %
    %
    %
    %[INTERNAL] 
    %Get symbolic primitives equivalent to the input expressions.
    %
    %There is no guarantee that subsequent calls return unique answers
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1x4
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L941
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L941-L943
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(755, self, varargin{:});
    end
    function varargout = mx_in(self,varargin)
    %MX_IN [INTERNAL] 
    %
    %  {MX} = MX_IN(self)
    %  MX = MX_IN(self, int ind)
    %  MX = MX_IN(self, char iname)
    %
    %Get symbolic primitives equivalent to the input expressions.
    %
    %There is no guarantee that subsequent calls return unique answers
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1x4
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L949
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1566-L1568
    %
    %
    %
    %.......
    %
    %::
    %
    %  MX_IN(self)
    %
    %
    %
    %[INTERNAL] 
    %Get symbolic primitives equivalent to the input expressions.
    %
    %There is no guarantee that subsequent calls return unique answers
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1x4
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L949
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1566-L1568
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  MX_IN(self, int ind)
    %
    %
    %
    %[INTERNAL] 
    %Get symbolic primitives equivalent to the input expressions.
    %
    %There is no guarantee that subsequent calls return unique answers
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1x4
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L945
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1558-L1560
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  MX_IN(self, char iname)
    %
    %
    %
    %[INTERNAL] 
    %Get symbolic primitives equivalent to the input expressions.
    %
    %There is no guarantee that subsequent calls return unique answers
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1x4
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L946
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L946-L948
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(756, self, varargin{:});
    end
    function varargout = sx_out(self,varargin)
    %SX_OUT [INTERNAL] 
    %
    %  {SX} = SX_OUT(self)
    %  SX = SX_OUT(self, int oind)
    %  SX = SX_OUT(self, char oname)
    %
    %Get symbolic primitives equivalent to the output expressions.
    %
    %There is no guarantee that subsequent calls return unique answers
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1x5
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L962
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1550-L1556
    %
    %
    %
    %.......
    %
    %::
    %
    %  SX_OUT(self)
    %
    %
    %
    %[INTERNAL] 
    %Get symbolic primitives equivalent to the output expressions.
    %
    %There is no guarantee that subsequent calls return unique answers
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1x5
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L962
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1550-L1556
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  SX_OUT(self, int oind)
    %
    %
    %
    %[INTERNAL] 
    %Get symbolic primitives equivalent to the output expressions.
    %
    %There is no guarantee that subsequent calls return unique answers
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1x5
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L958
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1534-L1540
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  SX_OUT(self, char oname)
    %
    %
    %
    %[INTERNAL] 
    %Get symbolic primitives equivalent to the output expressions.
    %
    %There is no guarantee that subsequent calls return unique answers
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1x5
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L959
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L959-L961
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(757, self, varargin{:});
    end
    function varargout = mx_out(self,varargin)
    %MX_OUT [INTERNAL] 
    %
    %  {MX} = MX_OUT(self)
    %  MX = MX_OUT(self, int ind)
    %  MX = MX_OUT(self, char oname)
    %
    %Get symbolic primitives equivalent to the output expressions.
    %
    %There is no guarantee that subsequent calls return unique answers
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1x5
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L967
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1570-L1572
    %
    %
    %
    %.......
    %
    %::
    %
    %  MX_OUT(self)
    %
    %
    %
    %[INTERNAL] 
    %Get symbolic primitives equivalent to the output expressions.
    %
    %There is no guarantee that subsequent calls return unique answers
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1x5
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L967
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1570-L1572
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  MX_OUT(self, int ind)
    %
    %
    %
    %[INTERNAL] 
    %Get symbolic primitives equivalent to the output expressions.
    %
    %There is no guarantee that subsequent calls return unique answers
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1x5
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L963
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1562-L1564
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  MX_OUT(self, char oname)
    %
    %
    %
    %[INTERNAL] 
    %Get symbolic primitives equivalent to the output expressions.
    %
    %There is no guarantee that subsequent calls return unique answers
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1x5
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L964
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L964-L966
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(758, self, varargin{:});
    end
    function varargout = nz_from_in(self,varargin)
    %NZ_FROM_IN [INTERNAL] 
    %
    %  [double] = NZ_FROM_IN(self, {DM} arg)
    %
    %Convert from/to flat vector of input/output nonzeros.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1x6
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L974
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1574-L1576
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(759, self, varargin{:});
    end
    function varargout = nz_from_out(self,varargin)
    %NZ_FROM_OUT [INTERNAL] 
    %
    %  [double] = NZ_FROM_OUT(self, {DM} arg)
    %
    %Convert from/to flat vector of input/output nonzeros.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1x6
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L975
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1578-L1580
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(760, self, varargin{:});
    end
    function varargout = nz_to_in(self,varargin)
    %NZ_TO_IN [INTERNAL] 
    %
    %  {DM} = NZ_TO_IN(self, [double] arg)
    %
    %Convert from/to flat vector of input/output nonzeros.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1x6
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L976
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1582-L1584
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(761, self, varargin{:});
    end
    function varargout = nz_to_out(self,varargin)
    %NZ_TO_OUT [INTERNAL] 
    %
    %  {DM} = NZ_TO_OUT(self, [double] arg)
    %
    %Convert from/to flat vector of input/output nonzeros.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1x6
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L977
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1586-L1588
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(762, self, varargin{:});
    end
    function varargout = convert_in(self,varargin)
    %CONVERT_IN [INTERNAL] 
    %
    %  {DM} = CONVERT_IN(self, struct:DM arg)
    %  struct:DM = CONVERT_IN(self, {DM} arg)
    %  struct:SX = CONVERT_IN(self, {SX} arg)
    %  {SX} = CONVERT_IN(self, struct:SX arg)
    %  {MX} = CONVERT_IN(self, struct:MX arg)
    %  struct:MX = CONVERT_IN(self, {MX} arg)
    %
    %Convert from/to input/output lists/map.
    %
    %Will raise an error when an unknown key is used or a list has 
    %incorrect 
    %size. Does not perform sparsity checking.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1x7
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L996
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1626-L1628
    %
    %
    %
    %.......
    %
    %::
    %
    %  CONVERT_IN(self, struct:DM arg)
    %
    %
    %
    %[INTERNAL] 
    %Convert from/to input/output lists/map.
    %
    %Will raise an error when an unknown key is used or a list has 
    %incorrect 
    %size. Does not perform sparsity checking.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1x7
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L988
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1594-L1596
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  CONVERT_IN(self, {DM} arg)
    %
    %
    %
    %[INTERNAL] 
    %Convert from/to input/output lists/map.
    %
    %Will raise an error when an unknown key is used or a list has 
    %incorrect 
    %size. Does not perform sparsity checking.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1x7
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L987
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1590-L1592
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  CONVERT_IN(self, {SX} arg)
    %
    %
    %
    %[INTERNAL] 
    %Convert from/to input/output lists/map.
    %
    %Will raise an error when an unknown key is used or a list has 
    %incorrect 
    %size. Does not perform sparsity checking.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1x7
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L991
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1606-L1608
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  CONVERT_IN(self, struct:SX arg)
    %
    %
    %
    %[INTERNAL] 
    %Convert from/to input/output lists/map.
    %
    %Will raise an error when an unknown key is used or a list has 
    %incorrect 
    %size. Does not perform sparsity checking.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1x7
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L992
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1610-L1612
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  CONVERT_IN(self, struct:MX arg)
    %
    %
    %
    %[INTERNAL] 
    %Convert from/to input/output lists/map.
    %
    %Will raise an error when an unknown key is used or a list has 
    %incorrect 
    %size. Does not perform sparsity checking.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1x7
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L996
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1626-L1628
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  CONVERT_IN(self, {MX} arg)
    %
    %
    %
    %[INTERNAL] 
    %Convert from/to input/output lists/map.
    %
    %Will raise an error when an unknown key is used or a list has 
    %incorrect 
    %size. Does not perform sparsity checking.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1x7
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L995
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1622-L1624
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(763, self, varargin{:});
    end
    function varargout = convert_out(self,varargin)
    %CONVERT_OUT [INTERNAL] 
    %
    %  {DM} = CONVERT_OUT(self, struct:DM arg)
    %  struct:DM = CONVERT_OUT(self, {DM} arg)
    %  struct:SX = CONVERT_OUT(self, {SX} arg)
    %  {SX} = CONVERT_OUT(self, struct:SX arg)
    %  {MX} = CONVERT_OUT(self, struct:MX arg)
    %  struct:MX = CONVERT_OUT(self, {MX} arg)
    %
    %Convert from/to input/output lists/map.
    %
    %Will raise an error when an unknown key is used or a list has 
    %incorrect 
    %size. Does not perform sparsity checking.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1x7
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L998
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1634-L1636
    %
    %
    %
    %.......
    %
    %::
    %
    %  CONVERT_OUT(self, struct:DM arg)
    %
    %
    %
    %[INTERNAL] 
    %Convert from/to input/output lists/map.
    %
    %Will raise an error when an unknown key is used or a list has 
    %incorrect 
    %size. Does not perform sparsity checking.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1x7
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L990
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1602-L1604
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  CONVERT_OUT(self, {DM} arg)
    %
    %
    %
    %[INTERNAL] 
    %Convert from/to input/output lists/map.
    %
    %Will raise an error when an unknown key is used or a list has 
    %incorrect 
    %size. Does not perform sparsity checking.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1x7
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L989
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1598-L1600
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  CONVERT_OUT(self, {SX} arg)
    %
    %
    %
    %[INTERNAL] 
    %Convert from/to input/output lists/map.
    %
    %Will raise an error when an unknown key is used or a list has 
    %incorrect 
    %size. Does not perform sparsity checking.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1x7
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L993
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1614-L1616
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  CONVERT_OUT(self, struct:SX arg)
    %
    %
    %
    %[INTERNAL] 
    %Convert from/to input/output lists/map.
    %
    %Will raise an error when an unknown key is used or a list has 
    %incorrect 
    %size. Does not perform sparsity checking.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1x7
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L994
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1618-L1620
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  CONVERT_OUT(self, struct:MX arg)
    %
    %
    %
    %[INTERNAL] 
    %Convert from/to input/output lists/map.
    %
    %Will raise an error when an unknown key is used or a list has 
    %incorrect 
    %size. Does not perform sparsity checking.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1x7
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L998
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1634-L1636
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  CONVERT_OUT(self, {MX} arg)
    %
    %
    %
    %[INTERNAL] 
    %Convert from/to input/output lists/map.
    %
    %Will raise an error when an unknown key is used or a list has 
    %incorrect 
    %size. Does not perform sparsity checking.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1x7
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L997
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1630-L1632
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(764, self, varargin{:});
    end
    function varargout = has_free(self,varargin)
    %HAS_FREE [INTERNAL] 
    %
    %  bool = HAS_FREE(self)
    %
    %Does the function have free variables.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1x8
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L1004
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1666-L1668
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(765, self, varargin{:});
    end
    function varargout = get_free(self,varargin)
    %GET_FREE [INTERNAL] 
    %
    %  {char} = GET_FREE(self)
    %
    %Get free variables as a string.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1x9
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L1009
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1159-L1161
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(766, self, varargin{:});
    end
    function varargout = free_sx(self,varargin)
    %FREE_SX [INTERNAL] 
    %
    %  {SX} = FREE_SX(self)
    %
    %Get all the free variables of the function.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1xa
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L1014
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1642-L1648
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(767, self, varargin{:});
    end
    function varargout = free_mx(self,varargin)
    %FREE_MX [INTERNAL] 
    %
    %  {MX} = FREE_MX(self)
    %
    %Get all the free variables of the function.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1xb
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L1019
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1650-L1656
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(768, self, varargin{:});
    end
    function varargout = generate_lifted(self,varargin)
    %GENERATE_LIFTED [INTERNAL] 
    %
    %  [Function OUTPUT, Function OUTPUT] = GENERATE_LIFTED(self)
    %
    %Extract the functions needed for the Lifted  Newton method.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1xc
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L1024
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1670-L1676
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(769, self, varargin{:});
    end
    function varargout = n_nodes(self,varargin)
    %N_NODES [INTERNAL] 
    %
    %  int = N_NODES(self)
    %
    %Number of nodes in the algorithm.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1xd
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L1030
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1734-L1740
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(770, self, varargin{:});
    end
    function varargout = n_instructions(self,varargin)
    %N_INSTRUCTIONS [INTERNAL] 
    %
    %  int = N_INSTRUCTIONS(self)
    %
    %Number of instruction in the algorithm (SXFunction/MXFunction)
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1xe
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L1035
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1678-L1684
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(771, self, varargin{:});
    end
    function varargout = instruction_id(self,varargin)
    %INSTRUCTION_ID [INTERNAL] 
    %
    %  int = INSTRUCTION_ID(self, int k)
    %
    %Identifier index of the instruction (SXFunction/MXFunction)
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1xf
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L1040
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1702-L1708
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(772, self, varargin{:});
    end
    function varargout = instruction_input(self,varargin)
    %INSTRUCTION_INPUT [INTERNAL] 
    %
    %  [int] = INSTRUCTION_INPUT(self, int k)
    %
    %Locations in the work vector for the inputs of the instruction.
    %
    %(SXFunction/MXFunction)
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1xg
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L1047
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1710-L1716
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(773, self, varargin{:});
    end
    function varargout = instruction_constant(self,varargin)
    %INSTRUCTION_CONSTANT [INTERNAL] 
    %
    %  double = INSTRUCTION_CONSTANT(self, int k)
    %
    %Get the floating point output argument of an instruction 
    %(SXFunction)
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1xh
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L1052
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1718-L1724
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(774, self, varargin{:});
    end
    function varargout = instruction_output(self,varargin)
    %INSTRUCTION_OUTPUT [INTERNAL] 
    %
    %  [int] = INSTRUCTION_OUTPUT(self, int k)
    %
    %Location in the work vector for the output of the instruction.
    %
    %(SXFunction/MXFunction)
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1xi
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L1059
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1726-L1732
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(775, self, varargin{:});
    end
    function varargout = instruction_MX(self,varargin)
    %INSTRUCTION_MX [INTERNAL] 
    %
    %  MX = INSTRUCTION_MX(self, int k)
    %
    %Get the  MX node corresponding to an instruction (MXFunction)
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1xj
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L1064
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1686-L1692
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(776, self, varargin{:});
    end
    function varargout = instructions_sx(self,varargin)
    %INSTRUCTIONS_SX [INTERNAL] 
    %
    %  SX = INSTRUCTIONS_SX(self)
    %
    %Get the SX node corresponding to all instructions (SXFunction)
    %
    %Note: input and output instructions have no SX representation. This 
    %method 
    %returns nan for those instructions.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1xk
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L1072
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1694-L1700
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(777, self, varargin{:});
    end
    function varargout = has_spfwd(self,varargin)
    %HAS_SPFWD [INTERNAL] 
    %
    %  bool = HAS_SPFWD(self)
    %
    %Is the class able to propagate seeds through the algorithm?
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1xl
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L1078
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1658-L1660
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(778, self, varargin{:});
    end
    function varargout = has_sprev(self,varargin)
    %HAS_SPREV [INTERNAL] 
    %
    %  bool = HAS_SPREV(self)
    %
    %Is the class able to propagate seeds through the algorithm?
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1xl
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L1079
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1662-L1664
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(779, self, varargin{:});
    end
    function varargout = sz_arg(self,varargin)
    %SZ_ARG [INTERNAL] 
    %
    %  size_t = SZ_ARG(self)
    %
    %Get required length of arg field.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1xm
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L1085
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1058-L1058
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(780, self, varargin{:});
    end
    function varargout = sz_res(self,varargin)
    %SZ_RES [INTERNAL] 
    %
    %  size_t = SZ_RES(self)
    %
    %Get required length of res field.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1xn
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L1090
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1060-L1060
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(781, self, varargin{:});
    end
    function varargout = sz_iw(self,varargin)
    %SZ_IW [INTERNAL] 
    %
    %  size_t = SZ_IW(self)
    %
    %Get required length of iw field.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1xo
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L1095
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1062-L1062
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(782, self, varargin{:});
    end
    function varargout = sz_w(self,varargin)
    %SZ_W [INTERNAL] 
    %
    %  size_t = SZ_W(self)
    %
    %Get required length of w field.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1xp
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L1100
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1064-L1064
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(783, self, varargin{:});
    end
    function varargout = name(self,varargin)
    %NAME [INTERNAL] 
    %
    %  char = NAME(self)
    %
    %Name of the function.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1xv
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L1137
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1281-L1288
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(784, self, varargin{:});
    end
    function varargout = is_a(self,varargin)
    %IS_A [INTERNAL] 
    %
    %  bool = IS_A(self, char type, bool recursive)
    %
    %Check if the function is of a particular type.
    %
    %Optionally check if name matches one of the base classes (default 
    %true)
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1xw
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L1144
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1638-L1640
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(785, self, varargin{:});
    end
    function varargout = assert_size_in(self,varargin)
    %ASSERT_SIZE_IN [INTERNAL] 
    %
    %  ASSERT_SIZE_IN(self, int i, int nrow, int ncol)
    %
    %Assert that an input dimension is equal so some given value.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L1189
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1754-L1760
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(790, self, varargin{:});
    end
    function varargout = assert_size_out(self,varargin)
    %ASSERT_SIZE_OUT [INTERNAL] 
    %
    %  ASSERT_SIZE_OUT(self, int i, int nrow, int ncol)
    %
    %Assert that an output dimension is equal so some given value.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L1192
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1762-L1767
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(791, self, varargin{:});
    end
    function varargout = assert_sparsity_out(self,varargin)
    %ASSERT_SPARSITY_OUT [INTERNAL] 
    %
    %  ASSERT_SPARSITY_OUT(self, int i, Sparsity sp, int n, bool allow_all_zero_sparse)
    %
    %Assert that an output sparsity is a multiple of some given 
    %sparsity.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L1195
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1769-L1778
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(792, self, varargin{:});
    end
    function varargout = checkout(self,varargin)
    %CHECKOUT [INTERNAL] 
    %
    %  int = CHECKOUT(self)
    %
    %Checkout a memory object.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L1199
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1742-L1744
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(793, self, varargin{:});
    end
    function varargout = release(self,varargin)
    %RELEASE [INTERNAL] 
    %
    %  RELEASE(self, int mem)
    %
    %Release a memory object.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L1202
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1746-L1748
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(794, self, varargin{:});
    end
    function varargout = cache(self,varargin)
    %CACHE [INTERNAL] 
    %
    %  struct = CACHE(self)
    %
    %Get all functions in the cache.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_26i
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L1212
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1804-L1811
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(795, self, varargin{:});
    end
    function varargout = get_function(self,varargin)
    %GET_FUNCTION [INTERNAL] 
    %
    %  {char} = GET_FUNCTION(self)
    %  Function = GET_FUNCTION(self, char name)
    %
    %Get a dependency function.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1y4
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L1222
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1822-L1828
    %
    %
    %
    %.......
    %
    %::
    %
    %  GET_FUNCTION(self, char name)
    %
    %
    %
    %[INTERNAL] 
    %Get a dependency function.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1y4
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L1222
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1822-L1828
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  GET_FUNCTION(self)
    %
    %
    %
    %[INTERNAL] 
    %Get a list of all functions.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1y3
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L1217
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1813-L1820
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(796, self, varargin{:});
    end
    function varargout = has_function(self,varargin)
    %HAS_FUNCTION [INTERNAL] 
    %
    %  bool = HAS_FUNCTION(self, char fname)
    %
    %Check if a particular dependency exists.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1y5
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L1227
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1830-L1837
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(797, self, varargin{:});
    end
    function varargout = find_functions(self,varargin)
    %FIND_FUNCTIONS [INTERNAL] 
    %
    %  {Function} = FIND_FUNCTIONS(self, int max_depth)
    %
    %Get all functions embedded in the expression graphs.
    %
    %Parameters:
    %-----------
    %
    %max_depth: 
    %Maximum depth - a negative number indicates no maximum
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1y6
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L1234
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1839-L1855
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(798, self, varargin{:});
    end
    function varargout = find_function(self,varargin)
    %FIND_FUNCTION [INTERNAL] 
    %
    %  Function = FIND_FUNCTION(self, char name, int max_depth)
    %
    %Get a specific function embedded in the expression graphs.
    %
    %Parameters:
    %-----------
    %
    %name: 
    %Name of function needed
    %
    %max_depth: 
    %Maximum depth - a negative number indicates no maximum
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1y7
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L1242
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1857-L1874
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(799, self, varargin{:});
    end
    function varargout = info(self,varargin)
    %INFO [INTERNAL] 
    %
    %  struct = INFO(self)
    %
    %Obtain information about function
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L1245
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L1903-L1905
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(800, self, varargin{:});
    end

     function s = saveobj(obj)
        try
            s.serialization = obj.serialize();
        catch exception
            warning(['Serializing of CasADi Function failed:' getReport(exception) ]);
            s = struct;
        end
     end
  
    function varargout = subsref(self,s)
      if numel(s)==1 && strcmp(s.type,'()')
        [varargout{1:nargout}]= paren(self, s.subs{:});
      else
        [varargout{1:nargout}] = builtin('subsref',self,s);
      end
   end
   function varargout = paren(self, varargin)
      if nargin==1 || (nargin>=2 && ischar(varargin{1}))
        % Named inputs: return struct
        assert(nargout<2, 'Syntax error');
        assert(mod(nargin,2)==1, 'Syntax error');
        arg = struct;
        for i=1:2:nargin-1
          assert(ischar(varargin{i}), 'Syntax error');
          arg.(varargin{i}) = varargin{i+1};
        end
        res = self.call(arg);
        varargout{1} = res;
      else
        % Ordered inputs: return variable number of outputs
        res = self.call(varargin);
        assert(nargout<=numel(res), 'Too many outputs');
        for i=1:max(min(1,numel(res)),nargout)
          varargout{i} = res{i};
        end
      end
    end
      function self = Function(varargin)
    %FUNCTION 
    %
    %  new_obj = FUNCTION()
    %  new_obj = FUNCTION(char fname)
    %  new_obj = FUNCTION(char name, {SX} ex_in, {SX} ex_out, struct opts)
    %  new_obj = FUNCTION(char name, {MX} ex_in, {MX} ex_out, struct opts)
    %  new_obj = FUNCTION(char name, struct:SX dict, {char} name_in, {char} name_out, struct opts)
    %  new_obj = FUNCTION(char name, struct:MX dict, {char} name_in, {char} name_out, struct opts)
    %  new_obj = FUNCTION(char name, {SX} ex_in, {SX} ex_out, {char} name_in, {char} name_out, struct opts)
    %  new_obj = FUNCTION(char name, {MX} ex_in, {MX} ex_out, {char} name_in, {char} name_out, struct opts)
    %
    %
    %.......
    %
    %::
    %
    %  FUNCTION(char name, {MX} ex_in, {MX} ex_out, struct opts)
    %
    %
    %
    %[INTERNAL] 
    %Construct an  MX function.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1v1
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L101
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L109-L113
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  FUNCTION(char name, struct:MX dict, {char} name_in, {char} name_out, struct opts)
    %
    %
    %
    %[INTERNAL] 
    %Construct an  MX function.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1v1
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L111
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L188-L192
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  FUNCTION(char name, {MX} ex_in, {MX} ex_out, {char} name_in, {char} name_out, struct opts)
    %
    %
    %
    %[INTERNAL] 
    %Construct an  MX function.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1v1
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L105
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L115-L121
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  FUNCTION()
    %
    %
    %
    %[INTERNAL] 
    %Default constructor, null pointer.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1uy
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L70
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L57-L58
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  FUNCTION(char name, {SX} ex_in, {SX} ex_out, struct opts)
    %
    %
    %
    %[INTERNAL] 
    %Construct an SX function.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1v0
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L81
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L95-L99
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  FUNCTION(char name, struct:SX dict, {char} name_in, {char} name_out, struct opts)
    %
    %
    %
    %[INTERNAL] 
    %Construct an SX function.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1v0
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L91
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L182-L186
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  FUNCTION(char name, {SX} ex_in, {SX} ex_out, {char} name_in, {char} name_out, struct opts)
    %
    %
    %
    %[INTERNAL] 
    %Construct an SX function.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1v0
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L85
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L101-L107
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  FUNCTION(char fname)
    %
    %
    %
    %[INTERNAL] 
    %Construct from a file.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1uz
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L75
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L91-L93
    %
    %
    %
    %.............
    %
    %
      self@casadi.SharedObject(SwigRef.Null);
      self@casadi.PrintableCommon(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(801, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
  end
  methods(Static)
    function varargout = type_name(varargin)
    %TYPE_NAME 
    %
    %  char = TYPE_NAME()
    %
    %
     [varargout{1:nargout}] = casadiMEX(691, varargin{:});
    end
    function varargout = jit(varargin)
    %JIT [INTERNAL] 
    %
    %  Function = JIT(char name, char body, {char} name_in, {char} name_out, struct opts)
    %  Function = JIT(char name, char body, {char} name_in, {char} name_out, {Sparsity} sparsity_in, {Sparsity} sparsity_out, struct opts)
    %
    %Create a just-in-time compiled function from a C language 
    %string.
    %
    %The names and sparsity patterns of all the inputs and outputs must be 
    %
    %provided. If sparsities are not provided, all inputs and outputs are 
    %
    %assumed to be scalar. Only specify the function body, assuming that 
    %input 
    %and output nonzeros are stored in arrays with the specified 
    %naming 
    %convension. The data type used is 'casadi_real', which is 
    %typically equal 
    %to 'double or another data type with the same API as 'double.
    %
    %Inputs may be null pointers. This means that the all entries are zero.
    % 
    %Outputs may be null points. This means that the corresponding result 
    %can be
    % ignored.
    %
    %If an error occurs in the evaluation, issue "return 1;";
    %
    %The final generated function will have a structure similar to:
    %
    %casadi_int fname(const casadi_real** arg, casadi_real** res, 
    %casadi_int* 
    %iw, casadi_real* w, void* mem) { const casadi_real *x1, 
    %*x2; casadi_real 
    %*r1, *r2; x1 = *arg++; x2 = *arg++; r1 = *res++; r2 =
    % *res++; 
    %<FUNCTION_BODY> return 0; }
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1v3
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L189
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L293-L305
    %
    %
    %
    %.......
    %
    %::
    %
    %  JIT(char name, char body, {char} name_in, {char} name_out, struct opts)
    %
    %
    %
    %[INTERNAL] 
    %Create a just-in-time compiled function from a C language 
    %string.
    %
    %The names and sparsity patterns of all the inputs and outputs must be 
    %
    %provided. If sparsities are not provided, all inputs and outputs are 
    %
    %assumed to be scalar. Only specify the function body, assuming that 
    %input 
    %and output nonzeros are stored in arrays with the specified 
    %naming 
    %convension. The data type used is 'casadi_real', which is 
    %typically equal 
    %to 'double or another data type with the same API as 'double.
    %
    %Inputs may be null pointers. This means that the all entries are zero.
    % 
    %Outputs may be null points. This means that the corresponding result 
    %can be
    % ignored.
    %
    %If an error occurs in the evaluation, issue "return 1;";
    %
    %The final generated function will have a structure similar to:
    %
    %casadi_int fname(const casadi_real** arg, casadi_real** res, 
    %casadi_int* 
    %iw, casadi_real* w, void* mem) { const casadi_real *x1, 
    %*x2; casadi_real 
    %*r1, *r2; x1 = *arg++; x2 = *arg++; r1 = *res++; r2 =
    % *res++; 
    %<FUNCTION_BODY> return 0; }
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1v3
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L185
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L284-L291
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  JIT(char name, char body, {char} name_in, {char} name_out, {Sparsity} sparsity_in, {Sparsity} sparsity_out, struct opts)
    %
    %
    %
    %[INTERNAL] 
    %Create a just-in-time compiled function from a C language 
    %string.
    %
    %The names and sparsity patterns of all the inputs and outputs must be 
    %
    %provided. If sparsities are not provided, all inputs and outputs are 
    %
    %assumed to be scalar. Only specify the function body, assuming that 
    %input 
    %and output nonzeros are stored in arrays with the specified 
    %naming 
    %convension. The data type used is 'casadi_real', which is 
    %typically equal 
    %to 'double or another data type with the same API as 'double.
    %
    %Inputs may be null pointers. This means that the all entries are zero.
    % 
    %Outputs may be null points. This means that the corresponding result 
    %can be
    % ignored.
    %
    %If an error occurs in the evaluation, issue "return 1;";
    %
    %The final generated function will have a structure similar to:
    %
    %casadi_int fname(const casadi_real** arg, casadi_real** res, 
    %casadi_int* 
    %iw, casadi_real* w, void* mem) { const casadi_real *x1, 
    %*x2; casadi_real 
    %*r1, *r2; x1 = *arg++; x2 = *arg++; r1 = *res++; r2 =
    % *res++; 
    %<FUNCTION_BODY> return 0; }
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1v3
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.hpp#L189
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/function.cpp#L293-L305
    %
    %
    %
    %.............
    %
    %
     [varargout{1:nargout}] = casadiMEX(692, varargin{:});
    end
    function varargout = conditional(varargin)
    %CONDITIONAL 
    %
    %  Function = CONDITIONAL(char name, Function f, struct opts)
    %  Function = CONDITIONAL(char name, {Function} f, Function f_def, struct opts)
    %
    %
     [varargout{1:nargout}] = casadiMEX(741, varargin{:});
    end
    function varargout = bspline(varargin)
    %BSPLINE 
    %
    %  Function = BSPLINE(char name, {[double]} knots, [double] coeffs, [int] degree, int m, struct opts)
    %
    %
     [varargout{1:nargout}] = casadiMEX(742, varargin{:});
    end
    function varargout = if_else(varargin)
    %IF_ELSE 
    %
    %  Function = IF_ELSE(char name, Function f_true, Function f_false, struct opts)
    %
    %
     [varargout{1:nargout}] = casadiMEX(743, varargin{:});
    end
    function varargout = check_name(varargin)
    %CHECK_NAME 
    %
    %  bool = CHECK_NAME(char name)
    %
    %
     [varargout{1:nargout}] = casadiMEX(786, varargin{:});
    end
    function varargout = fix_name(varargin)
    %FIX_NAME 
    %
    %  char = FIX_NAME(char name)
    %
    %
     [varargout{1:nargout}] = casadiMEX(787, varargin{:});
    end
    function varargout = load(varargin)
    %LOAD 
    %
    %  Function = LOAD(char filename)
    %
    %
     [varargout{1:nargout}] = casadiMEX(788, varargin{:});
    end
    function varargout = deserialize(varargin)
    %DESERIALIZE 
    %
    %  Function = DESERIALIZE(std::istream & stream)
    %  Function = DESERIALIZE(casadi::DeserializingStream & s)
    %  Function = DESERIALIZE(char s)
    %
    %
     [varargout{1:nargout}] = casadiMEX(789, varargin{:});
    end

     function obj = loadobj(s)
        try
          if isstruct(s)
             obj = casadi.Function.deserialize(s.serialization);
          else
             obj = s;
          end
        catch exception
            warning(['Serializing of CasADi Function failed:' getReport(exception) ]);
            s = struct;
        end
     end
    end
end
