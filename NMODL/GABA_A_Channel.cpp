/* Created by Language version: 7.7.0 */
/* NOT VECTORIZED */
#define NRN_VECTORIZED 0
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "mech_api.h"
#undef PI
#define nil 0
#define _pval pval
// clang-format off
#include "md1redef.h"
#include "section_fwd.hpp"
#include "nrniv_mf.h"
#include "md2redef.h"
#include "nrnconf.h"
// clang-format on
#include "neuron/cache/mechanism_range.hpp"
#include <vector>
using std::size_t;
static auto& std_cerr_stream = std::cerr;
static constexpr auto number_of_datum_variables = 5;
static constexpr auto number_of_floating_point_variables = 4;
namespace {
template <typename T>
using _nrn_mechanism_std_vector = std::vector<T>;
using _nrn_model_sorted_token = neuron::model_sorted_token;
using _nrn_mechanism_cache_range = neuron::cache::MechanismRange<number_of_floating_point_variables, number_of_datum_variables>;
using _nrn_mechanism_cache_instance = neuron::cache::MechanismInstance<number_of_floating_point_variables, number_of_datum_variables>;
using _nrn_non_owning_id_without_container = neuron::container::non_owning_identifier_without_container;
template <typename T>
using _nrn_mechanism_field = neuron::mechanism::field<T>;
template <typename... Args>
void _nrn_mechanism_register_data_fields(Args&&... args) {
  neuron::mechanism::register_data_fields(std::forward<Args>(args)...);
}
}
 
#if !NRNGPU
#undef exp
#define exp hoc_Exp
#if NRN_ENABLE_ARCH_INDEP_EXP_POW
#undef pow
#define pow hoc_pow
#endif
#endif
 
#define nrn_init _nrn_init__GABA_A_Channel
#define _nrn_initial _nrn_initial__GABA_A_Channel
#define nrn_cur _nrn_cur__GABA_A_Channel
#define _nrn_current _nrn_current__GABA_A_Channel
#define nrn_jacob _nrn_jacob__GABA_A_Channel
#define nrn_state _nrn_state__GABA_A_Channel
#define _net_receive _net_receive__GABA_A_Channel 
 
#define _threadargscomma_ /**/
#define _threadargsprotocomma_ /**/
#define _internalthreadargsprotocomma_ /**/
#define _threadargs_ /**/
#define _threadargsproto_ /**/
#define _internalthreadargsproto_ /**/
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *hoc_getarg(int);
 
#define t nrn_threads->_t
#define dt nrn_threads->_dt
#define gmax _ml->template fpfield<0>(_iml)
#define gmax_columnindex 0
#define ecl _ml->template fpfield<1>(_iml)
#define ecl_columnindex 1
#define icl _ml->template fpfield<2>(_iml)
#define icl_columnindex 2
#define _g _ml->template fpfield<3>(_iml)
#define _g_columnindex 3
#define _nd_area *_ml->dptr_field<0>(_iml)
#define _ion_icl *(_ml->dptr_field<2>(_iml))
#define _p_ion_icl static_cast<neuron::container::data_handle<double>>(_ppvar[2])
#define _ion_dicldv *(_ml->dptr_field<3>(_iml))
#define receptor_activation	*_ppvar[4].get<double*>()
#define _p_receptor_activation _ppvar[4].literal_value<void*>()
 static _nrn_mechanism_cache_instance _ml_real{nullptr};
static _nrn_mechanism_cache_range *_ml{&_ml_real};
static size_t _iml{0};
static Datum *_ppvar;
 static int hoc_nrnpointerindex =  4;
 /* external NEURON variables */
 /* declaration of user functions */
 static int _mechtype;
extern void _nrn_cacheloop_reg(int, int);
extern void hoc_register_limits(int, HocParmLimits*);
extern void hoc_register_units(int, HocParmUnits*);
extern void nrn_promote(Prop*, int, int);
 
#define NMODL_TEXT 1
#if NMODL_TEXT
static void register_nmodl_text_and_filename(int mechtype);
#endif
 extern Prop* nrn_point_prop_;
 static int _pointtype;
 static void* _hoc_create_pnt(Object* _ho) { void* create_point_process(int, Object*);
 return create_point_process(_pointtype, _ho);
}
 static void _hoc_destroy_pnt(void*);
 static double _hoc_loc_pnt(void* _vptr) {double loc_point_process(int, void*);
 return loc_point_process(_pointtype, _vptr);
}
 static double _hoc_has_loc(void* _vptr) {double has_loc_point(void*);
 return has_loc_point(_vptr);
}
 static double _hoc_get_loc_pnt(void* _vptr) {
 double get_loc_point_process(void*); return (get_loc_point_process(_vptr));
}
 static void _hoc_setdata(void*);
 /* connect user functions to hoc names */
 static VoidFunc hoc_intfunc[] = {
 {0, 0}
};
 static Member_func _member_func[] = {
 {"loc", _hoc_loc_pnt},
 {"has_loc", _hoc_has_loc},
 {"get_loc", _hoc_get_loc_pnt},
 {0, 0}
};
 /* declare global and static user variables */
 #define gind 0
 #define _gth 0
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 {0, 0, 0}
};
 static HocParmUnits _hoc_parm_units[] = {
 {"gmax", "uS"},
 {"ecl", "mV"},
 {"receptor_activation", "1"},
 {0, 0}
};
 static double v = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 {0, 0}
};
 static DoubVec hoc_vdoub[] = {
 {0, 0, 0}
};
 static double _sav_indep;
 extern void _nrn_setdata_reg(int, void(*)(Prop*));
 static void _setdata(Prop* _prop) {
 neuron::legacy::set_globals_from_prop(_prop, _ml_real, _ml, _iml);
_ppvar = _nrn_mechanism_access_dparam(_prop);
 Node * _node = _nrn_mechanism_access_node(_prop);
v = _nrn_mechanism_access_voltage(_node);
 }
 static void _hoc_setdata(void* _vptr) { Prop* _prop;
 _prop = ((Point_process*)_vptr)->_prop;
   _setdata(_prop);
 }
 static void nrn_alloc(Prop*);
static void nrn_init(_nrn_model_sorted_token const&, NrnThread*, Memb_list*, int);
static void nrn_state(_nrn_model_sorted_token const&, NrnThread*, Memb_list*, int);
 static void nrn_cur(_nrn_model_sorted_token const&, NrnThread*, Memb_list*, int);
static void nrn_jacob(_nrn_model_sorted_token const&, NrnThread*, Memb_list*, int);
 static void _hoc_destroy_pnt(void* _vptr) {
   destroy_point_process(_vptr);
}
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"GABA_A_Channel",
 "gmax",
 "ecl",
 0,
 0,
 0,
 "receptor_activation",
 0};
 static Symbol* _cl_sym;
 
 /* Used by NrnProperty */
 static _nrn_mechanism_std_vector<double> _parm_default{
     0.1, /* gmax */
     -70, /* ecl */
 }; 
 
 
extern Prop* need_memb(Symbol*);
static void nrn_alloc(Prop* _prop) {
  Prop *prop_ion{};
  Datum *_ppvar{};
  if (nrn_point_prop_) {
    _nrn_mechanism_access_alloc_seq(_prop) = _nrn_mechanism_access_alloc_seq(nrn_point_prop_);
    _ppvar = _nrn_mechanism_access_dparam(nrn_point_prop_);
  } else {
   _ppvar = nrn_prop_datum_alloc(_mechtype, 5, _prop);
    _nrn_mechanism_access_dparam(_prop) = _ppvar;
     _nrn_mechanism_cache_instance _ml_real{_prop};
    auto* const _ml = &_ml_real;
    size_t const _iml{};
    assert(_nrn_mechanism_get_num_vars(_prop) == 4);
 	/*initialize range parameters*/
 	gmax = _parm_default[0]; /* 0.1 */
 	ecl = _parm_default[1]; /* -70 */
  }
 	 assert(_nrn_mechanism_get_num_vars(_prop) == 4);
 	_nrn_mechanism_access_dparam(_prop) = _ppvar;
 	/*connect ionic variables to this model*/
 prop_ion = need_memb(_cl_sym);
 	_ppvar[2] = _nrn_mechanism_get_param_handle(prop_ion, 3); /* icl */
 	_ppvar[3] = _nrn_mechanism_get_param_handle(prop_ion, 4); /* _ion_dicldv */
 
}
 static void _initlists();
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
void _nrn_thread_table_reg(int, nrn_thread_table_check_t);
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 extern "C" void _GABA_A_Channel_reg() {
	int _vectorized = 0;
  _initlists();
 	ion_reg("cl", -1.0);
 	_cl_sym = hoc_lookup("cl_ion");
 	_pointtype = point_register_mech(_mechanism,
	 nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init,
	 hoc_nrnpointerindex, 0,
	 _hoc_create_pnt, _hoc_destroy_pnt, _member_func);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
 hoc_register_parm_default(_mechtype, &_parm_default);
     _nrn_setdata_reg(_mechtype, _setdata);
 #if NMODL_TEXT
  register_nmodl_text_and_filename(_mechtype);
#endif
   _nrn_mechanism_register_data_fields(_mechtype,
                                       _nrn_mechanism_field<double>{"gmax"} /* 0 */,
                                       _nrn_mechanism_field<double>{"ecl"} /* 1 */,
                                       _nrn_mechanism_field<double>{"icl"} /* 2 */,
                                       _nrn_mechanism_field<double>{"_g"} /* 3 */,
                                       _nrn_mechanism_field<double*>{"_nd_area", "area"} /* 0 */,
                                       _nrn_mechanism_field<Point_process*>{"_pntproc", "pntproc"} /* 1 */,
                                       _nrn_mechanism_field<double*>{"_ion_icl", "cl_ion"} /* 2 */,
                                       _nrn_mechanism_field<double*>{"_ion_dicldv", "cl_ion"} /* 3 */,
                                       _nrn_mechanism_field<double*>{"receptor_activation", "pointer"} /* 4 */);
  hoc_register_prop_size(_mechtype, 4, 5);
  hoc_register_dparam_semantics(_mechtype, 0, "area");
  hoc_register_dparam_semantics(_mechtype, 1, "pntproc");
  hoc_register_dparam_semantics(_mechtype, 2, "cl_ion");
  hoc_register_dparam_semantics(_mechtype, 3, "cl_ion");
  hoc_register_dparam_semantics(_mechtype, 4, "pointer");
 
    hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 GABA_A_Channel C\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static const char *modelname = "GABA_A_Channel";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}

static void initmodel() {
  int _i; double _save;_ninits++;
{

}
}

static void nrn_init(_nrn_model_sorted_token const& _sorted_token, NrnThread* _nt, Memb_list* _ml_arg, int _type){
Node *_nd; double _v; int* _ni; int _cntml;
_nrn_mechanism_cache_range _lmr{_sorted_token, *_nt, *_ml_arg, _type};
auto* const _vec_v = _nt->node_voltage_storage();
_ml = &_lmr;
_ni = _ml_arg->_nodeindices;
_cntml = _ml_arg->_nodecount;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _ppvar = _ml_arg->_pdata[_iml];
   _v = _vec_v[_ni[_iml]];
 v = _v;
 initmodel();
 }}

static double _nrn_current(double _v){double _current=0.;v=_v;{ {
   icl = gmax * receptor_activation * ( v - ecl ) ;
   }
 _current += icl;

} return _current;
}

static void nrn_cur(_nrn_model_sorted_token const& _sorted_token, NrnThread* _nt, Memb_list* _ml_arg, int _type){
_nrn_mechanism_cache_range _lmr{_sorted_token, *_nt, *_ml_arg, _type};
auto const _vec_rhs = _nt->node_rhs_storage();
auto const _vec_sav_rhs = _nt->node_sav_rhs_storage();
auto const _vec_v = _nt->node_voltage_storage();
Node *_nd; int* _ni; double _rhs, _v; int _cntml;
_ml = &_lmr;
_ni = _ml_arg->_nodeindices;
_cntml = _ml_arg->_nodecount;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _ppvar = _ml_arg->_pdata[_iml];
   _v = _vec_v[_ni[_iml]];
 auto const _g_local = _nrn_current(_v + .001);
 	{ double _dicl;
  _dicl = icl;
 _rhs = _nrn_current(_v);
  _ion_dicldv += (_dicl - icl)/.001 * 1.e2/ (_nd_area);
 	}
 _g = (_g_local - _rhs)/.001;
  _ion_icl += icl * 1.e2/ (_nd_area);
 _g *=  1.e2/(_nd_area);
 _rhs *= 1.e2/(_nd_area);
	 _vec_rhs[_ni[_iml]] -= _rhs;
 
}}

static void nrn_jacob(_nrn_model_sorted_token const& _sorted_token, NrnThread* _nt, Memb_list* _ml_arg, int _type) {
_nrn_mechanism_cache_range _lmr{_sorted_token, *_nt, *_ml_arg, _type};
auto const _vec_d = _nt->node_d_storage();
auto const _vec_sav_d = _nt->node_sav_d_storage();
auto* const _ml = &_lmr;
Node *_nd; int* _ni; int _iml, _cntml;
_ni = _ml_arg->_nodeindices;
_cntml = _ml_arg->_nodecount;
for (_iml = 0; _iml < _cntml; ++_iml) {
  _vec_d[_ni[_iml]] += _g;
 
}}

static void nrn_state(_nrn_model_sorted_token const& _sorted_token, NrnThread* _nt, Memb_list* _ml_arg, int _type){

}

static void terminal(){}

static void _initlists() {
 int _i; static int _first = 1;
  if (!_first) return;
_first = 0;
}

#if NMODL_TEXT
static void register_nmodl_text_and_filename(int mech_type) {
    const char* nmodl_filename = "C";
    const char* nmodl_file_text = 
  "TITLE GABA_A_Channel\n"
  "COMMENT\n"
  "A simple GABA_A receptor mechanism that:\n"
  " - Uses receptor activation as input\n"
  " - Allows chloride (Cl-) current\n"
  "ENDCOMMENT\n"
  "\n"
  "NEURON {\n"
  "    POINT_PROCESS GABA_A_Channel\n"
  "    USEION cl WRITE icl VALENCE -1  : Define chloride as a negatively charged ion\n"
  "    RANGE gmax, ecl\n"
  "    POINTER receptor_activation\n"
  "}\n"
  "\n"
  "PARAMETER {\n"
  "    gmax = 0.1 (uS)    : Maximum conductance\n"
  "    ecl  = -70 (mV)    : Reversal potential for chloride\n"
  "}\n"
  "\n"
  "ASSIGNED {\n"
  "    v      (mV)        : Membrane potential\n"
  "    receptor_activation (1)\n"
  "    icl    (nA)        : Chloride current\n"
  "}\n"
  "\n"
  "BREAKPOINT {\n"
  "    : Chloride current\n"
  "    icl = gmax * receptor_activation * (v - ecl)\n"
  "}\n"
  ;
    hoc_reg_nmodl_filename(mech_type, nmodl_filename);
    hoc_reg_nmodl_text(mech_type, nmodl_file_text);
}
#endif
