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
static constexpr auto number_of_datum_variables = 3;
static constexpr auto number_of_floating_point_variables = 14;
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
 
#define nrn_init _nrn_init__NMDA_Channel_Calcium
#define _nrn_initial _nrn_initial__NMDA_Channel_Calcium
#define nrn_cur _nrn_cur__NMDA_Channel_Calcium
#define _nrn_current _nrn_current__NMDA_Channel_Calcium
#define nrn_jacob _nrn_jacob__NMDA_Channel_Calcium
#define nrn_state _nrn_state__NMDA_Channel_Calcium
#define _net_receive _net_receive__NMDA_Channel_Calcium 
#define state state__NMDA_Channel_Calcium 
 
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
#define e _ml->template fpfield<1>(_iml)
#define e_columnindex 1
#define depth _ml->template fpfield<2>(_iml)
#define depth_columnindex 2
#define taur _ml->template fpfield<3>(_iml)
#define taur_columnindex 3
#define mg _ml->template fpfield<4>(_iml)
#define mg_columnindex 4
#define alpha_mg _ml->template fpfield<5>(_iml)
#define alpha_mg_columnindex 5
#define beta_mg _ml->template fpfield<6>(_iml)
#define beta_mg_columnindex 6
#define i _ml->template fpfield<7>(_iml)
#define i_columnindex 7
#define mgblock _ml->template fpfield<8>(_iml)
#define mgblock_columnindex 8
#define local_cai _ml->template fpfield<9>(_iml)
#define local_cai_columnindex 9
#define g _ml->template fpfield<10>(_iml)
#define g_columnindex 10
#define drive_ca _ml->template fpfield<11>(_iml)
#define drive_ca_columnindex 11
#define Dlocal_cai _ml->template fpfield<12>(_iml)
#define Dlocal_cai_columnindex 12
#define _g _ml->template fpfield<13>(_iml)
#define _g_columnindex 13
#define _nd_area *_ml->dptr_field<0>(_iml)
#define receptor_activation	*_ppvar[2].get<double*>()
#define _p_receptor_activation _ppvar[2].literal_value<void*>()
 static _nrn_mechanism_cache_instance _ml_real{nullptr};
static _nrn_mechanism_cache_range *_ml{&_ml_real};
static size_t _iml{0};
static Datum *_ppvar;
 static int hoc_nrnpointerindex =  2;
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
 {"e", "mV"},
 {"depth", "um"},
 {"taur", "ms"},
 {"mg", "mM"},
 {"alpha_mg", "/mV"},
 {"local_cai", "mM"},
 {"i", "nA"},
 {"receptor_activation", "1"},
 {0, 0}
};
 static double delta_t = 0.01;
 static double local_cai0 = 0;
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
 
static int _ode_count(int);
static void _ode_map(Prop*, int, neuron::container::data_handle<double>*, neuron::container::data_handle<double>*, double*, int);
static void _ode_spec(_nrn_model_sorted_token const&, NrnThread*, Memb_list*, int);
static void _ode_matsol(_nrn_model_sorted_token const&, NrnThread*, Memb_list*, int);
 
#define _cvode_ieq _ppvar[3].literal_value<int>()
 static void _ode_matsol_instance1(_internalthreadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"NMDA_Channel_Calcium",
 "gmax",
 "e",
 "depth",
 "taur",
 "mg",
 "alpha_mg",
 "beta_mg",
 0,
 "i",
 "mgblock",
 0,
 "local_cai",
 0,
 "receptor_activation",
 0};
 
 /* Used by NrnProperty */
 static _nrn_mechanism_std_vector<double> _parm_default{
     0.1, /* gmax */
     0, /* e */
     0.1, /* depth */
     200, /* taur */
     1, /* mg */
     0.062, /* alpha_mg */
     3.57, /* beta_mg */
 }; 
 
 
extern Prop* need_memb(Symbol*);
static void nrn_alloc(Prop* _prop) {
  Prop *prop_ion{};
  Datum *_ppvar{};
  if (nrn_point_prop_) {
    _nrn_mechanism_access_alloc_seq(_prop) = _nrn_mechanism_access_alloc_seq(nrn_point_prop_);
    _ppvar = _nrn_mechanism_access_dparam(nrn_point_prop_);
  } else {
   _ppvar = nrn_prop_datum_alloc(_mechtype, 4, _prop);
    _nrn_mechanism_access_dparam(_prop) = _ppvar;
     _nrn_mechanism_cache_instance _ml_real{_prop};
    auto* const _ml = &_ml_real;
    size_t const _iml{};
    assert(_nrn_mechanism_get_num_vars(_prop) == 14);
 	/*initialize range parameters*/
 	gmax = _parm_default[0]; /* 0.1 */
 	e = _parm_default[1]; /* 0 */
 	depth = _parm_default[2]; /* 0.1 */
 	taur = _parm_default[3]; /* 200 */
 	mg = _parm_default[4]; /* 1 */
 	alpha_mg = _parm_default[5]; /* 0.062 */
 	beta_mg = _parm_default[6]; /* 3.57 */
  }
 	 assert(_nrn_mechanism_get_num_vars(_prop) == 14);
 	_nrn_mechanism_access_dparam(_prop) = _ppvar;
 	/*connect ionic variables to this model*/
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 {0, 0}
};
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
void _nrn_thread_table_reg(int, nrn_thread_table_check_t);
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 extern "C" void _NMDA_Channel_Calcium_reg() {
	int _vectorized = 0;
  _initlists();
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
                                       _nrn_mechanism_field<double>{"e"} /* 1 */,
                                       _nrn_mechanism_field<double>{"depth"} /* 2 */,
                                       _nrn_mechanism_field<double>{"taur"} /* 3 */,
                                       _nrn_mechanism_field<double>{"mg"} /* 4 */,
                                       _nrn_mechanism_field<double>{"alpha_mg"} /* 5 */,
                                       _nrn_mechanism_field<double>{"beta_mg"} /* 6 */,
                                       _nrn_mechanism_field<double>{"i"} /* 7 */,
                                       _nrn_mechanism_field<double>{"mgblock"} /* 8 */,
                                       _nrn_mechanism_field<double>{"local_cai"} /* 9 */,
                                       _nrn_mechanism_field<double>{"g"} /* 10 */,
                                       _nrn_mechanism_field<double>{"drive_ca"} /* 11 */,
                                       _nrn_mechanism_field<double>{"Dlocal_cai"} /* 12 */,
                                       _nrn_mechanism_field<double>{"_g"} /* 13 */,
                                       _nrn_mechanism_field<double*>{"_nd_area", "area"} /* 0 */,
                                       _nrn_mechanism_field<Point_process*>{"_pntproc", "pntproc"} /* 1 */,
                                       _nrn_mechanism_field<double*>{"receptor_activation", "pointer"} /* 2 */,
                                       _nrn_mechanism_field<int>{"_cvode_ieq", "cvodeieq"} /* 3 */);
  hoc_register_prop_size(_mechtype, 14, 4);
  hoc_register_dparam_semantics(_mechtype, 0, "area");
  hoc_register_dparam_semantics(_mechtype, 1, "pntproc");
  hoc_register_dparam_semantics(_mechtype, 2, "pointer");
  hoc_register_dparam_semantics(_mechtype, 3, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 
    hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 NMDA_Channel_Calcium C\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
 static double FARADAY = 96485.3;
static int _reset;
static const char *modelname = "NMDA Receptor with Calcium Dynamics";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
 
static int _ode_spec1(_internalthreadargsproto_);
/*static int _ode_matsol1(_internalthreadargsproto_);*/
 static neuron::container::field_index _slist1[1], _dlist1[1];
 static int state(_internalthreadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 () {_reset=0;
 {
   Dlocal_cai = drive_ca - local_cai / taur ;
   }
 return _reset;
}
 static int _ode_matsol1 () {
 Dlocal_cai = Dlocal_cai  / (1. - dt*( ( - ( 1.0 ) / taur ) )) ;
  return 0;
}
 /*END CVODE*/
 static int state () {_reset=0;
 {
    local_cai = local_cai + (1. - exp(dt*(( - ( 1.0 ) / taur ))))*(- ( drive_ca ) / ( ( - ( 1.0 ) / taur ) ) - local_cai) ;
   }
  return 0;
}
 
static int _ode_count(int _type){ return 1;}
 
static void _ode_spec(_nrn_model_sorted_token const& _sorted_token, NrnThread* _nt, Memb_list* _ml_arg, int _type) {
      Node* _nd{};
  double _v{};
  int _cntml;
  _nrn_mechanism_cache_range _lmr{_sorted_token, *_nt, *_ml_arg, _type};
  _ml = &_lmr;
  _cntml = _ml_arg->_nodecount;
  Datum *_thread{_ml_arg->_thread};
  double* _globals = nullptr;
  if (gind != 0 && _thread != nullptr) { _globals = _thread[_gth].get<double*>(); }
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _ppvar = _ml_arg->_pdata[_iml];
    _nd = _ml_arg->_nodelist[_iml];
    v = NODEV(_nd);
     _ode_spec1 ();
 }}
 
static void _ode_map(Prop* _prop, int _ieq, neuron::container::data_handle<double>* _pv, neuron::container::data_handle<double>* _pvdot, double* _atol, int _type) { 
  _ppvar = _nrn_mechanism_access_dparam(_prop);
  _cvode_ieq = _ieq;
  for (int _i=0; _i < 1; ++_i) {
    _pv[_i] = _nrn_mechanism_get_param_handle(_prop, _slist1[_i]);
    _pvdot[_i] = _nrn_mechanism_get_param_handle(_prop, _dlist1[_i]);
    _cvode_abstol(_atollist, _atol, _i);
  }
 }
 
static void _ode_matsol_instance1(_internalthreadargsproto_) {
 _ode_matsol1 ();
 }
 
static void _ode_matsol(_nrn_model_sorted_token const& _sorted_token, NrnThread* _nt, Memb_list* _ml_arg, int _type) {
      Node* _nd{};
  double _v{};
  int _cntml;
  _nrn_mechanism_cache_range _lmr{_sorted_token, *_nt, *_ml_arg, _type};
  _ml = &_lmr;
  _cntml = _ml_arg->_nodecount;
  Datum *_thread{_ml_arg->_thread};
  double* _globals = nullptr;
  if (gind != 0 && _thread != nullptr) { _globals = _thread[_gth].get<double*>(); }
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _ppvar = _ml_arg->_pdata[_iml];
    _nd = _ml_arg->_nodelist[_iml];
    v = NODEV(_nd);
 _ode_matsol_instance1(_threadargs_);
 }}

static void initmodel() {
  int _i; double _save;_ninits++;
 _save = t;
 t = 0.0;
{
  local_cai = local_cai0;
  _sav_indep = t; t = _save;

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
   mgblock = 1.0 / ( 1.0 + mg * beta_mg * exp ( - alpha_mg * v ) ) ;
   g = gmax * receptor_activation * mgblock ;
   i = g * ( v - e ) ;
   drive_ca = - g * ( v - 140.0 ) / ( 2.0 * FARADAY * depth ) ;
   if ( drive_ca <= 0.0 ) {
     drive_ca = 0.0 ;
     }
   }
 _current += i;

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
 	{ _rhs = _nrn_current(_v);
 	}
 _g = (_g_local - _rhs)/.001;
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
Node *_nd; double _v = 0.0; int* _ni; int _cntml;
_nrn_mechanism_cache_range _lmr{_sorted_token, *_nt, *_ml_arg, _type};
auto* const _vec_v = _nt->node_voltage_storage();
_ml = &_lmr;
_ni = _ml_arg->_nodeindices;
_cntml = _ml_arg->_nodecount;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _ppvar = _ml_arg->_pdata[_iml];
 _nd = _ml_arg->_nodelist[_iml];
   _v = _vec_v[_ni[_iml]];
 v=_v;
{
 { error =  state();
 if(error){
  std_cerr_stream << "at line 37 in file NMDA_Channel_Calcium.mod:\nBREAKPOINT {\n";
  std_cerr_stream << _ml << ' ' << _iml << '\n';
  abort_run(error);
}
 }}}

}

static void terminal(){}

static void _initlists() {
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = {local_cai_columnindex, 0};  _dlist1[0] = {Dlocal_cai_columnindex, 0};
_first = 0;
}

#if NMODL_TEXT
static void register_nmodl_text_and_filename(int mech_type) {
    const char* nmodl_filename = "C";
    const char* nmodl_file_text = 
  "TITLE NMDA Receptor with Calcium Dynamics\n"
  "\n"
  "NEURON {\n"
  "    POINT_PROCESS NMDA_Channel_Calcium\n"
  "    NONSPECIFIC_CURRENT i\n"
  "    RANGE gmax, e, mgblock, mg, alpha_mg, beta_mg, depth, taur, local_cai\n"
  "    POINTER receptor_activation\n"
  "}\n"
  "\n"
  "CONSTANT {\n"
  "    FARADAY = 96485.3 (coulomb/mol) : Faraday's constant\n"
  "}\n"
  "\n"
  "PARAMETER {\n"
  "    gmax = 0.1 (uS)          : Maximum total conductance\n"
  "    e = 0 (mV)               : Reversal potential\n"
  "    depth = 0.1 (um)         : Shell depth for calcium accumulation\n"
  "    taur = 200 (ms)          : Calcium decay time constant\n"
  "    mg = 1 (mM)              : External [Mg2+]\n"
  "    alpha_mg = 0.062 (/mV)   : Voltage-dependence factor\n"
  "    beta_mg = 3.57           : Mg2+ block scaling\n"
  "}\n"
  "\n"
  "ASSIGNED {\n"
  "    v (mV)                   : Membrane potential\n"
  "    i (nA)                   : Nonspecific current\n"
  "    g (uS)                   : Total channel conductance\n"
  "    mgblock                  : Magnesium block factor\n"
  "    drive_ca (mM/ms)         : Calcium influx rate\n"
  "    receptor_activation (1)\n"
  "}\n"
  "\n"
  "STATE {\n"
  "    local_cai (mM)           : Localized calcium concentration\n"
  "}\n"
  "\n"
  "BREAKPOINT {\n"
  "    SOLVE state METHOD cnexp\n"
  "\n"
  "    : Calculate Mg2+ block\n"
  "    mgblock = 1 / (1 + mg * beta_mg * exp(-alpha_mg * v))\n"
  "\n"
  "    : Total conductance\n"
  "    g = gmax * receptor_activation * mgblock\n"
  "\n"
  "    : Nonspecific current\n"
  "    i = g * (v - e)\n"
  "\n"
  "    : Calcium influx rate\n"
  "    drive_ca = -g * (v - 140) / (2 * FARADAY * depth)\n"
  "\n"
  "    : Prevent unphysical calcium extrusion\n"
  "    if (drive_ca <= 0) {\n"
  "        drive_ca = 0\n"
  "    }\n"
  "}\n"
  "\n"
  "DERIVATIVE state {\n"
  "    : Localized calcium dynamics\n"
  "    local_cai' = drive_ca - local_cai / taur\n"
  "}\n"
  ;
    hoc_reg_nmodl_filename(mech_type, nmodl_filename);
    hoc_reg_nmodl_text(mech_type, nmodl_file_text);
}
#endif
