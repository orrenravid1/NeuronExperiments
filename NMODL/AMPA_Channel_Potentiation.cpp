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
static constexpr auto number_of_datum_variables = 8;
static constexpr auto number_of_floating_point_variables = 23;
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
 
#define nrn_init _nrn_init__AMPA_Channel_Potentiation
#define _nrn_initial _nrn_initial__AMPA_Channel_Potentiation
#define nrn_cur _nrn_cur__AMPA_Channel_Potentiation
#define _nrn_current _nrn_current__AMPA_Channel_Potentiation
#define nrn_jacob _nrn_jacob__AMPA_Channel_Potentiation
#define nrn_state _nrn_state__AMPA_Channel_Potentiation
#define _net_receive _net_receive__AMPA_Channel_Potentiation 
#define state state__AMPA_Channel_Potentiation 
 
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
#define baseline_gmax _ml->template fpfield<0>(_iml)
#define baseline_gmax_columnindex 0
#define ena _ml->template fpfield<1>(_iml)
#define ena_columnindex 1
#define ek _ml->template fpfield<2>(_iml)
#define ek_columnindex 2
#define p_ratio _ml->template fpfield<3>(_iml)
#define p_ratio_columnindex 3
#define potentiation_threshold _ml->template fpfield<4>(_iml)
#define potentiation_threshold_columnindex 4
#define depression_threshold _ml->template fpfield<5>(_iml)
#define depression_threshold_columnindex 5
#define potentiation_rate _ml->template fpfield<6>(_iml)
#define potentiation_rate_columnindex 6
#define depression_rate _ml->template fpfield<7>(_iml)
#define depression_rate_columnindex 7
#define potentiation_strength _ml->template fpfield<8>(_iml)
#define potentiation_strength_columnindex 8
#define depression_strength _ml->template fpfield<9>(_iml)
#define depression_strength_columnindex 9
#define calcium_baseline _ml->template fpfield<10>(_iml)
#define calcium_baseline_columnindex 10
#define potentiation_decay _ml->template fpfield<11>(_iml)
#define potentiation_decay_columnindex 11
#define depression_decay _ml->template fpfield<12>(_iml)
#define depression_decay_columnindex 12
#define P _ml->template fpfield<13>(_iml)
#define P_columnindex 13
#define D _ml->template fpfield<14>(_iml)
#define D_columnindex 14
#define ina _ml->template fpfield<15>(_iml)
#define ina_columnindex 15
#define ik _ml->template fpfield<16>(_iml)
#define ik_columnindex 16
#define gna _ml->template fpfield<17>(_iml)
#define gna_columnindex 17
#define gk _ml->template fpfield<18>(_iml)
#define gk_columnindex 18
#define gmax _ml->template fpfield<19>(_iml)
#define gmax_columnindex 19
#define DP _ml->template fpfield<20>(_iml)
#define DP_columnindex 20
#define DD _ml->template fpfield<21>(_iml)
#define DD_columnindex 21
#define _g _ml->template fpfield<22>(_iml)
#define _g_columnindex 22
#define _nd_area *_ml->dptr_field<0>(_iml)
#define _ion_ina *(_ml->dptr_field<2>(_iml))
#define _p_ion_ina static_cast<neuron::container::data_handle<double>>(_ppvar[2])
#define _ion_dinadv *(_ml->dptr_field<3>(_iml))
#define _ion_ik *(_ml->dptr_field<4>(_iml))
#define _p_ion_ik static_cast<neuron::container::data_handle<double>>(_ppvar[4])
#define _ion_dikdv *(_ml->dptr_field<5>(_iml))
#define receptor_activation	*_ppvar[6].get<double*>()
#define _p_receptor_activation _ppvar[6].literal_value<void*>()
#define local_cai	*_ppvar[7].get<double*>()
#define _p_local_cai _ppvar[7].literal_value<void*>()
 static _nrn_mechanism_cache_instance _ml_real{nullptr};
static _nrn_mechanism_cache_range *_ml{&_ml_real};
static size_t _iml{0};
static Datum *_ppvar;
 static int hoc_nrnpointerindex =  6;
 /* external NEURON variables */
 /* declaration of user functions */
 static double _hoc_max(void*);
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
 {"max", _hoc_max},
 {0, 0}
};
#define max max_AMPA_Channel_Potentiation
 extern double max( double , double );
 /* declare global and static user variables */
 #define gind 0
 #define _gth 0
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 {0, 0, 0}
};
 static HocParmUnits _hoc_parm_units[] = {
 {"baseline_gmax", "uS"},
 {"ena", "mV"},
 {"ek", "mV"},
 {"potentiation_threshold", "mM"},
 {"depression_threshold", "mM"},
 {"potentiation_rate", "/ms"},
 {"depression_rate", "/ms"},
 {"potentiation_strength", "1"},
 {"depression_strength", "1"},
 {"calcium_baseline", "mM"},
 {"potentiation_decay", "1"},
 {"depression_decay", "1"},
 {"P", "1"},
 {"D", "1"},
 {"receptor_activation", "1"},
 {"local_cai", "mM"},
 {0, 0}
};
 static double D0 = 0;
 static double P0 = 0;
 static double delta_t = 0.01;
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
 
#define _cvode_ieq _ppvar[8].literal_value<int>()
 static void _ode_matsol_instance1(_internalthreadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"AMPA_Channel_Potentiation",
 "baseline_gmax",
 "ena",
 "ek",
 "p_ratio",
 "potentiation_threshold",
 "depression_threshold",
 "potentiation_rate",
 "depression_rate",
 "potentiation_strength",
 "depression_strength",
 "calcium_baseline",
 "potentiation_decay",
 "depression_decay",
 0,
 0,
 "P",
 "D",
 0,
 "receptor_activation",
 "local_cai",
 0};
 static Symbol* _na_sym;
 static Symbol* _k_sym;
 
 /* Used by NrnProperty */
 static _nrn_mechanism_std_vector<double> _parm_default{
     0.1, /* baseline_gmax */
     50, /* ena */
     -90, /* ek */
     10, /* p_ratio */
     0.5, /* potentiation_threshold */
     0.5, /* depression_threshold */
     0.01, /* potentiation_rate */
     0.005, /* depression_rate */
     1, /* potentiation_strength */
     1, /* depression_strength */
     0.0001, /* calcium_baseline */
     0.1, /* potentiation_decay */
     0.1, /* depression_decay */
 }; 
 
 
extern Prop* need_memb(Symbol*);
static void nrn_alloc(Prop* _prop) {
  Prop *prop_ion{};
  Datum *_ppvar{};
  if (nrn_point_prop_) {
    _nrn_mechanism_access_alloc_seq(_prop) = _nrn_mechanism_access_alloc_seq(nrn_point_prop_);
    _ppvar = _nrn_mechanism_access_dparam(nrn_point_prop_);
  } else {
   _ppvar = nrn_prop_datum_alloc(_mechtype, 9, _prop);
    _nrn_mechanism_access_dparam(_prop) = _ppvar;
     _nrn_mechanism_cache_instance _ml_real{_prop};
    auto* const _ml = &_ml_real;
    size_t const _iml{};
    assert(_nrn_mechanism_get_num_vars(_prop) == 23);
 	/*initialize range parameters*/
 	baseline_gmax = _parm_default[0]; /* 0.1 */
 	ena = _parm_default[1]; /* 50 */
 	ek = _parm_default[2]; /* -90 */
 	p_ratio = _parm_default[3]; /* 10 */
 	potentiation_threshold = _parm_default[4]; /* 0.5 */
 	depression_threshold = _parm_default[5]; /* 0.5 */
 	potentiation_rate = _parm_default[6]; /* 0.01 */
 	depression_rate = _parm_default[7]; /* 0.005 */
 	potentiation_strength = _parm_default[8]; /* 1 */
 	depression_strength = _parm_default[9]; /* 1 */
 	calcium_baseline = _parm_default[10]; /* 0.0001 */
 	potentiation_decay = _parm_default[11]; /* 0.1 */
 	depression_decay = _parm_default[12]; /* 0.1 */
  }
 	 assert(_nrn_mechanism_get_num_vars(_prop) == 23);
 	_nrn_mechanism_access_dparam(_prop) = _ppvar;
 	/*connect ionic variables to this model*/
 prop_ion = need_memb(_na_sym);
 	_ppvar[2] = _nrn_mechanism_get_param_handle(prop_ion, 3); /* ina */
 	_ppvar[3] = _nrn_mechanism_get_param_handle(prop_ion, 4); /* _ion_dinadv */
 prop_ion = need_memb(_k_sym);
 	_ppvar[4] = _nrn_mechanism_get_param_handle(prop_ion, 3); /* ik */
 	_ppvar[5] = _nrn_mechanism_get_param_handle(prop_ion, 4); /* _ion_dikdv */
 
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

 extern "C" void _AMPA_Channel_Potentiation_reg() {
	int _vectorized = 0;
  _initlists();
 	ion_reg("na", 1.0);
 	ion_reg("k", 1.0);
 	_na_sym = hoc_lookup("na_ion");
 	_k_sym = hoc_lookup("k_ion");
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
                                       _nrn_mechanism_field<double>{"baseline_gmax"} /* 0 */,
                                       _nrn_mechanism_field<double>{"ena"} /* 1 */,
                                       _nrn_mechanism_field<double>{"ek"} /* 2 */,
                                       _nrn_mechanism_field<double>{"p_ratio"} /* 3 */,
                                       _nrn_mechanism_field<double>{"potentiation_threshold"} /* 4 */,
                                       _nrn_mechanism_field<double>{"depression_threshold"} /* 5 */,
                                       _nrn_mechanism_field<double>{"potentiation_rate"} /* 6 */,
                                       _nrn_mechanism_field<double>{"depression_rate"} /* 7 */,
                                       _nrn_mechanism_field<double>{"potentiation_strength"} /* 8 */,
                                       _nrn_mechanism_field<double>{"depression_strength"} /* 9 */,
                                       _nrn_mechanism_field<double>{"calcium_baseline"} /* 10 */,
                                       _nrn_mechanism_field<double>{"potentiation_decay"} /* 11 */,
                                       _nrn_mechanism_field<double>{"depression_decay"} /* 12 */,
                                       _nrn_mechanism_field<double>{"P"} /* 13 */,
                                       _nrn_mechanism_field<double>{"D"} /* 14 */,
                                       _nrn_mechanism_field<double>{"ina"} /* 15 */,
                                       _nrn_mechanism_field<double>{"ik"} /* 16 */,
                                       _nrn_mechanism_field<double>{"gna"} /* 17 */,
                                       _nrn_mechanism_field<double>{"gk"} /* 18 */,
                                       _nrn_mechanism_field<double>{"gmax"} /* 19 */,
                                       _nrn_mechanism_field<double>{"DP"} /* 20 */,
                                       _nrn_mechanism_field<double>{"DD"} /* 21 */,
                                       _nrn_mechanism_field<double>{"_g"} /* 22 */,
                                       _nrn_mechanism_field<double*>{"_nd_area", "area"} /* 0 */,
                                       _nrn_mechanism_field<Point_process*>{"_pntproc", "pntproc"} /* 1 */,
                                       _nrn_mechanism_field<double*>{"_ion_ina", "na_ion"} /* 2 */,
                                       _nrn_mechanism_field<double*>{"_ion_dinadv", "na_ion"} /* 3 */,
                                       _nrn_mechanism_field<double*>{"_ion_ik", "k_ion"} /* 4 */,
                                       _nrn_mechanism_field<double*>{"_ion_dikdv", "k_ion"} /* 5 */,
                                       _nrn_mechanism_field<double*>{"receptor_activation", "pointer"} /* 6 */,
                                       _nrn_mechanism_field<double*>{"local_cai", "pointer"} /* 7 */,
                                       _nrn_mechanism_field<int>{"_cvode_ieq", "cvodeieq"} /* 8 */);
  hoc_register_prop_size(_mechtype, 23, 9);
  hoc_register_dparam_semantics(_mechtype, 0, "area");
  hoc_register_dparam_semantics(_mechtype, 1, "pntproc");
  hoc_register_dparam_semantics(_mechtype, 2, "na_ion");
  hoc_register_dparam_semantics(_mechtype, 3, "na_ion");
  hoc_register_dparam_semantics(_mechtype, 4, "k_ion");
  hoc_register_dparam_semantics(_mechtype, 5, "k_ion");
  hoc_register_dparam_semantics(_mechtype, 6, "pointer");
  hoc_register_dparam_semantics(_mechtype, 7, "pointer");
  hoc_register_dparam_semantics(_mechtype, 8, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 
    hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 AMPA_Channel_Potentiation C\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static const char *modelname = "AMPA_Channel with Calcium Dependent Potentiation and Depression";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
 
static int _ode_spec1(_internalthreadargsproto_);
/*static int _ode_matsol1(_internalthreadargsproto_);*/
 static neuron::container::field_index _slist1[2], _dlist1[2];
 static int state(_internalthreadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 () {_reset=0;
 {
   double _lrelative_calcium ;
 _lrelative_calcium = local_cai / calcium_baseline ;
   DP = potentiation_rate * max ( _threadargscomma_ 0.0 , _lrelative_calcium - potentiation_threshold ) * ( 1.0 - P ) - potentiation_decay * P ;
   DD = depression_rate * max ( _threadargscomma_ 0.0 , depression_threshold - _lrelative_calcium ) * ( 1.0 - D ) - depression_decay * D ;
   }
 return _reset;
}
 static int _ode_matsol1 () {
 double _lrelative_calcium ;
 _lrelative_calcium = local_cai / calcium_baseline ;
 DP = DP  / (1. - dt*( ( potentiation_rate * max ( _threadargscomma_ 0.0 , _lrelative_calcium - potentiation_threshold ) )*( ( ( - 1.0 ) ) ) - ( potentiation_decay )*( 1.0 ) )) ;
 DD = DD  / (1. - dt*( ( depression_rate * max ( _threadargscomma_ 0.0 , depression_threshold - _lrelative_calcium ) )*( ( ( - 1.0 ) ) ) - ( depression_decay )*( 1.0 ) )) ;
  return 0;
}
 /*END CVODE*/
 static int state () {_reset=0;
 {
   double _lrelative_calcium ;
 _lrelative_calcium = local_cai / calcium_baseline ;
    P = P + (1. - exp(dt*(( potentiation_rate * max ( _threadargscomma_ 0.0 , _lrelative_calcium - potentiation_threshold ) )*( ( ( - 1.0 ) ) ) - ( potentiation_decay )*( 1.0 ))))*(- ( ( ( potentiation_rate )*( max ( _threadargscomma_ 0.0 , _lrelative_calcium - potentiation_threshold ) ) )*( ( 1.0 ) ) ) / ( ( ( potentiation_rate )*( max ( _threadargscomma_ 0.0 , _lrelative_calcium - potentiation_threshold ) ) )*( ( ( - 1.0 ) ) ) - ( potentiation_decay )*( 1.0 ) ) - P) ;
    D = D + (1. - exp(dt*(( depression_rate * max ( _threadargscomma_ 0.0 , depression_threshold - _lrelative_calcium ) )*( ( ( - 1.0 ) ) ) - ( depression_decay )*( 1.0 ))))*(- ( ( ( depression_rate )*( max ( _threadargscomma_ 0.0 , depression_threshold - _lrelative_calcium ) ) )*( ( 1.0 ) ) ) / ( ( ( depression_rate )*( max ( _threadargscomma_ 0.0 , depression_threshold - _lrelative_calcium ) ) )*( ( ( - 1.0 ) ) ) - ( depression_decay )*( 1.0 ) ) - D) ;
   }
  return 0;
}
 
double max (  double _lx1 , double _lx2 ) {
   double _lmax;
 if ( _lx1 >= _lx2 ) {
     _lmax = _lx1 ;
     }
   else {
     _lmax = _lx2 ;
     }
   
return _lmax;
 }
 
static double _hoc_max(void* _vptr) {
 double _r;
    auto* const _pnt = static_cast<Point_process*>(_vptr);
  auto* const _p = _pnt->_prop;
  if (!_p) {
    hoc_execerror("POINT_PROCESS data instance not valid", NULL);
  }
   _setdata(_p);
 _r =  max (  *getarg(1) , *getarg(2) );
 return(_r);
}
 
static int _ode_count(int _type){ return 2;}
 
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
  for (int _i=0; _i < 2; ++_i) {
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
  D = D0;
  P = P0;
 {
   P = 0.0 ;
   D = 0.0 ;
   gmax = baseline_gmax ;
   }
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
   gmax = max ( _threadargscomma_ 0.0 , baseline_gmax * ( 1.0 + P * potentiation_strength - D * depression_strength ) ) ;
   gna = gmax * receptor_activation * p_ratio / ( 1.0 + p_ratio ) ;
   gk = gmax * receptor_activation / ( 1.0 + p_ratio ) ;
   ina = gna * ( v - ena ) ;
   ik = gk * ( v - ek ) ;
   }
 _current += ina;
 _current += ik;

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
 	{ double _dik;
 double _dina;
  _dina = ina;
  _dik = ik;
 _rhs = _nrn_current(_v);
  _ion_dinadv += (_dina - ina)/.001 * 1.e2/ (_nd_area);
  _ion_dikdv += (_dik - ik)/.001 * 1.e2/ (_nd_area);
 	}
 _g = (_g_local - _rhs)/.001;
  _ion_ina += ina * 1.e2/ (_nd_area);
  _ion_ik += ik * 1.e2/ (_nd_area);
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
  std_cerr_stream << "at line 51 in file AMPA_Channel_Potentiation.mod:\nBREAKPOINT {\n";
  std_cerr_stream << _ml << ' ' << _iml << '\n';
  abort_run(error);
}
 }  }}

}

static void terminal(){}

static void _initlists() {
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = {P_columnindex, 0};  _dlist1[0] = {DP_columnindex, 0};
 _slist1[1] = {D_columnindex, 0};  _dlist1[1] = {DD_columnindex, 0};
_first = 0;
}

#if NMODL_TEXT
static void register_nmodl_text_and_filename(int mech_type) {
    const char* nmodl_filename = "C";
    const char* nmodl_file_text = 
  "TITLE AMPA_Channel with Calcium Dependent Potentiation and Depression\n"
  "\n"
  "NEURON {\n"
  "    POINT_PROCESS AMPA_Channel_Potentiation\n"
  "    USEION na WRITE ina VALENCE 1\n"
  "    USEION k WRITE ik VALENCE 1\n"
  "    RANGE baseline_gmax, ena, ek, p_ratio\n"
  "    RANGE potentiation_threshold, depression_threshold, potentiation_rate, potentiation_strength, depression_rate, depression_strength, P, D\n"
  "    RANGE calcium_baseline, potentiation_decay, depression_decay\n"
  "    POINTER receptor_activation, local_cai\n"
  "}\n"
  "\n"
  "PARAMETER {\n"
  "    receptor_activation (1)\n"
  "    baseline_gmax = 0.1 (uS)   : Baseline maximum conductance\n"
  "    ena = 50 (mV)              : Sodium reversal potential\n"
  "    ek = -90 (mV)              : Potassium reversal potential\n"
  "    p_ratio = 10               : Sodium-to-potassium permeability ratio\n"
  "    potentiation_threshold = 0.5 (mM) : Calcium threshold for potentiation\n"
  "    depression_threshold = 0.5 (mM) : Calcium threshold for depression\n"
  "    potentiation_rate = 0.01 (/ms)  : Potentiation rate\n"
  "    depression_rate = 0.005 (/ms)   : Depression rate\n"
  "    potentiation_strength = 1 (1)  : Multiplier for potentiation\n"
  "    depression_strength = 1 (1) : Multiplier for depression\n"
  "    calcium_baseline = 0.0001 (mM)\n"
  "    potentiation_decay = 0.1 (1)\n"
  "    depression_decay = 0.1 (1)\n"
  "}\n"
  "\n"
  "ASSIGNED {\n"
  "    v (mV)                     : Membrane potential\n"
  "    local_cai (mM)             : Local calcium concentration\n"
  "    ina (nA)                   : Sodium current\n"
  "    ik (nA)                    : Potassium current\n"
  "    gna (uS)                   : Sodium conductance\n"
  "    gk (uS)                    : Potassium conductance\n"
  "    gmax (uS)                  : Dynamic maximum conductance\n"
  "}\n"
  "\n"
  "STATE {\n"
  "    P (1)                      : Potentiation factor (scaling gmax)\n"
  "    D (1)                      : Depression factor (scaling gmax)\n"
  "}\n"
  "\n"
  "INITIAL {\n"
  "    P = 0                     : No potentiation initially\n"
  "    D = 0                     : No depression initially\n"
  "    gmax = baseline_gmax       : Initial gmax\n"
  "}\n"
  "\n"
  "BREAKPOINT {\n"
  "    SOLVE state METHOD cnexp\n"
  "\n"
  "    : Scale gmax by potentiation factor\n"
  "    gmax = max(0, baseline_gmax * (1 + P * potentiation_strength - D * depression_strength))\n"
  "\n"
  "    : Calculate sodium and potassium conductances using permeability ratio\n"
  "    gna = gmax * receptor_activation * p_ratio / (1 + p_ratio)\n"
  "    gk = gmax * receptor_activation / (1 + p_ratio)\n"
  "\n"
  "    : Calculate sodium and potassium currents\n"
  "    ina = gna * (v - ena)\n"
  "    ik = gk * (v - ek)\n"
  "}\n"
  "\n"
  "DERIVATIVE state {\n"
  "    LOCAL relative_calcium\n"
  "\n"
  "    : Compute relative calcium\n"
  "    relative_calcium = local_cai / calcium_baseline\n"
  "\n"
  "    : Potentiation dynamics (smooth thresholding for calcium)\n"
  "    P' = potentiation_rate * max(0, relative_calcium - potentiation_threshold) * (1 - P)\n"
  "       - potentiation_decay * P\n"
  "\n"
  "    : Depression dynamics (smooth thresholding for calcium)\n"
  "    : Only unrealistic thing here is that depression can happen at 0 calcium\n"
  "    D' = depression_rate * max(0, depression_threshold - relative_calcium) * (1 - D)\n"
  "       - depression_decay * D\n"
  "}\n"
  "\n"
  "FUNCTION max(x1, x2) {\n"
  "    if (x1 >= x2){\n"
  "        max = x1\n"
  "    }\n"
  "    else{\n"
  "        max = x2\n"
  "    }\n"
  "}\n"
  ;
    hoc_reg_nmodl_filename(mech_type, nmodl_filename);
    hoc_reg_nmodl_text(mech_type, nmodl_file_text);
}
#endif
