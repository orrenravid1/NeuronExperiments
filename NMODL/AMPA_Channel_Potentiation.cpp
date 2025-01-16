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
#include "md1redef.h"
#include "section.h"
#include "nrniv_mf.h"
#include "md2redef.h"
 
#if !NRNGPU
#undef exp
#define exp hoc_Exp
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
#define _threadargs_ /**/
#define _threadargsproto_ /**/
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *hoc_getarg(int);
 static double *_p; static Datum *_ppvar;
 
#define t nrn_threads->_t
#define dt nrn_threads->_dt
#define baseline_gmax _p[0]
#define baseline_gmax_columnindex 0
#define ena _p[1]
#define ena_columnindex 1
#define ek _p[2]
#define ek_columnindex 2
#define p_ratio _p[3]
#define p_ratio_columnindex 3
#define calcium_threshold _p[4]
#define calcium_threshold_columnindex 4
#define potentiation_rate _p[5]
#define potentiation_rate_columnindex 5
#define decay_rate _p[6]
#define decay_rate_columnindex 6
#define potentiation_strength _p[7]
#define potentiation_strength_columnindex 7
#define P _p[8]
#define P_columnindex 8
#define ina _p[9]
#define ina_columnindex 9
#define ik _p[10]
#define ik_columnindex 10
#define gna _p[11]
#define gna_columnindex 11
#define gk _p[12]
#define gk_columnindex 12
#define gmax _p[13]
#define gmax_columnindex 13
#define DP _p[14]
#define DP_columnindex 14
#define _g _p[15]
#define _g_columnindex 15
#define _nd_area  *_ppvar[0].get<double*>()
#define _ion_ina	*_ppvar[2].get<double*>()
#define _ion_dinadv	*_ppvar[3].get<double*>()
#define _ion_ik	*_ppvar[4].get<double*>()
#define _ion_dikdv	*_ppvar[5].get<double*>()
#define receptor_activation	*_ppvar[6].get<double*>()
#define _p_receptor_activation _ppvar[6].literal_value<void*>()
#define NMDA_cai	*_ppvar[7].get<double*>()
#define _p_NMDA_cai _ppvar[7].literal_value<void*>()
 
#if MAC
#if !defined(v)
#define v _mlhv
#endif
#if !defined(h)
#define h _mlhh
#endif
#endif
 static int hoc_nrnpointerindex =  6;
 /* external NEURON variables */
 /* declaration of user functions */
 static int _mechtype;
extern void _nrn_cacheloop_reg(int, int);
extern void hoc_register_prop_size(int, int, int);
extern void hoc_register_limits(int, HocParmLimits*);
extern void hoc_register_units(int, HocParmUnits*);
extern void nrn_promote(Prop*, int, int);
extern Memb_func* memb_func;
 
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
 extern void _nrn_setdata_reg(int, void(*)(Prop*));
 static void _setdata(Prop* _prop) {
 _p = _prop->param; _ppvar = _prop->dparam;
 }
 static void _hoc_setdata(void* _vptr) { Prop* _prop;
 _prop = ((Point_process*)_vptr)->_prop;
   _setdata(_prop);
 }
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
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 {0, 0, 0}
};
 static HocParmUnits _hoc_parm_units[] = {
 {"baseline_gmax", "uS"},
 {"ena", "mV"},
 {"ek", "mV"},
 {"calcium_threshold", "mM"},
 {"potentiation_rate", "/ms"},
 {"decay_rate", "/ms"},
 {"potentiation_strength", "1"},
 {"P", "1"},
 {"receptor_activation", "1"},
 {"NMDA_cai", "mM"},
 {0, 0}
};
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
 static void nrn_alloc(Prop*);
static void  nrn_init(NrnThread*, Memb_list*, int);
static void nrn_state(NrnThread*, Memb_list*, int);
 static void nrn_cur(NrnThread*, Memb_list*, int);
static void  nrn_jacob(NrnThread*, Memb_list*, int);
 static void _hoc_destroy_pnt(void* _vptr) {
   destroy_point_process(_vptr);
}
 
static int _ode_count(int);
static void _ode_map(int, double**, double**, double*, Datum*, double*, int);
static void _ode_spec(NrnThread*, Memb_list*, int);
static void _ode_matsol(NrnThread*, Memb_list*, int);
 
#define _cvode_ieq _ppvar[8].literal_value<int>()
 static void _ode_matsol_instance1(_threadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"AMPA_Channel_Potentiation",
 "baseline_gmax",
 "ena",
 "ek",
 "p_ratio",
 "calcium_threshold",
 "potentiation_rate",
 "decay_rate",
 "potentiation_strength",
 0,
 0,
 "P",
 0,
 "receptor_activation",
 "NMDA_cai",
 0};
 static Symbol* _na_sym;
 static Symbol* _k_sym;
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
  if (nrn_point_prop_) {
	_prop->_alloc_seq = nrn_point_prop_->_alloc_seq;
	_p = nrn_point_prop_->param;
	_ppvar = nrn_point_prop_->dparam;
 }else{
 	_p = nrn_prop_data_alloc(_mechtype, 16, _prop);
 	/*initialize range parameters*/
 	baseline_gmax = 0.1;
 	ena = 50;
 	ek = -90;
 	p_ratio = 10;
 	calcium_threshold = 0.0001;
 	potentiation_rate = 0.01;
 	decay_rate = 0.005;
 	potentiation_strength = 1;
  }
 	_prop->param = _p;
 	_prop->param_size = 16;
  if (!nrn_point_prop_) {
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 9, _prop);
  }
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 prop_ion = need_memb(_na_sym);
 	_ppvar[2] = &prop_ion->param[3]; /* ina */
 	_ppvar[3] = &prop_ion->param[4]; /* _ion_dinadv */
 prop_ion = need_memb(_k_sym);
 	_ppvar[4] = &prop_ion->param[3]; /* ik */
 	_ppvar[5] = &prop_ion->param[4]; /* _ion_dikdv */
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 {0, 0}
};
 static void _update_ion_pointer(Datum*);
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, NrnThread*, int));
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
     _nrn_setdata_reg(_mechtype, _setdata);
     _nrn_thread_reg(_mechtype, 2, _update_ion_pointer);
 #if NMODL_TEXT
  register_nmodl_text_and_filename(_mechtype);
#endif
  hoc_register_prop_size(_mechtype, 16, 9);
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
 	ivoc_help("help ?1 AMPA_Channel_Potentiation AMPA_Channel_Potentiation.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static const char *modelname = "AMPA_Channel with Calcium Dependent Potentiation";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static int _slist1[2], _dlist1[2];
 static int state(_threadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 () {_reset=0;
 {
   if ( NMDA_cai > calcium_threshold ) {
     DP = potentiation_rate * NMDA_cai ;
     }
   else {
     DP = - decay_rate * P ;
     }
   }
 return _reset;
}
 static int _ode_matsol1 () {
 if ( NMDA_cai > calcium_threshold ) {
   DP = DP  / (1. - dt*( 0.0 )) ;
   }
 else {
   DP = DP  / (1. - dt*( ( - decay_rate )*( 1.0 ) )) ;
   }
  return 0;
}
 /*END CVODE*/
 static int state () {_reset=0;
 {
   if ( NMDA_cai > calcium_threshold ) {
      P = P - dt*(- ( ( potentiation_rate )*( NMDA_cai ) ) ) ;
     }
   else {
      P = P + (1. - exp(dt*(( - decay_rate )*( 1.0 ))))*(- ( 0.0 ) / ( ( - decay_rate )*( 1.0 ) ) - P) ;
     }
   }
  return 0;
}
 
static int _ode_count(int _type){ return 2;}
 
static void _ode_spec(NrnThread* _nt, Memb_list* _ml, int _type) {
   Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
     _ode_spec1 ();
   }}
 
static void _ode_map(int _ieq, double** _pv, double** _pvdot, double* _pp, Datum* _ppd, double* _atol, int _type) { 
 	int _i; _p = _pp; _ppvar = _ppd;
	_cvode_ieq = _ieq;
	for (_i=0; _i < 2; ++_i) {
		_pv[_i] = _pp + _slist1[_i];  _pvdot[_i] = _pp + _dlist1[_i];
		_cvode_abstol(_atollist, _atol, _i);
	}
 }
 
static void _ode_matsol_instance1(_threadargsproto_) {
 _ode_matsol1 ();
 }
 
static void _ode_matsol(NrnThread* _nt, Memb_list* _ml, int _type) {
   Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
 _ode_matsol_instance1(_threadargs_);
 }}
 extern void nrn_update_ion_pointer(Symbol*, Datum*, int, int);
 static void _update_ion_pointer(Datum* _ppvar) {
   nrn_update_ion_pointer(_na_sym, _ppvar, 2, 3);
   nrn_update_ion_pointer(_na_sym, _ppvar, 3, 4);
   nrn_update_ion_pointer(_k_sym, _ppvar, 4, 3);
   nrn_update_ion_pointer(_k_sym, _ppvar, 5, 4);
 }

static void initmodel() {
  int _i; double _save;_ninits++;
 _save = t;
 t = 0.0;
{
  P = P0;
 {
   P = 0.0 ;
   gmax = baseline_gmax ;
   }
  _sav_indep = t; t = _save;

}
}

static void nrn_init(NrnThread* _nt, Memb_list* _ml, int _type){
Node *_nd; double _v; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v = _v;
 initmodel();
  }}

static double _nrn_current(double _v){double _current=0.;v=_v;{ {
   gmax = baseline_gmax * ( 1.0 + P * potentiation_strength ) ;
   gna = gmax * receptor_activation * p_ratio / ( 1.0 + p_ratio ) ;
   gk = gmax * receptor_activation / ( 1.0 + p_ratio ) ;
   ina = gna * ( v - ena ) ;
   ik = gk * ( v - ek ) ;
   }
 _current += ina;
 _current += ik;

} return _current;
}

static void nrn_cur(NrnThread* _nt, Memb_list* _ml, int _type){
Node *_nd; int* _ni; double _rhs, _v; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 _g = _nrn_current(_v + .001);
 	{ double _dik;
 double _dina;
  _dina = ina;
  _dik = ik;
 _rhs = _nrn_current(_v);
  _ion_dinadv += (_dina - ina)/.001 * 1.e2/ (_nd_area);
  _ion_dikdv += (_dik - ik)/.001 * 1.e2/ (_nd_area);
 	}
 _g = (_g - _rhs)/.001;
  _ion_ina += ina * 1.e2/ (_nd_area);
  _ion_ik += ik * 1.e2/ (_nd_area);
 _g *=  1.e2/(_nd_area);
 _rhs *= 1.e2/(_nd_area);
#if CACHEVEC
  if (use_cachevec) {
	VEC_RHS(_ni[_iml]) -= _rhs;
  }else
#endif
  {
	NODERHS(_nd) -= _rhs;
  }
 
}}

static void nrn_jacob(NrnThread* _nt, Memb_list* _ml, int _type){
Node *_nd; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml];
#if CACHEVEC
  if (use_cachevec) {
	VEC_D(_ni[_iml]) += _g;
  }else
#endif
  {
     _nd = _ml->_nodelist[_iml];
	NODED(_nd) += _g;
  }
 
}}

static void nrn_state(NrnThread* _nt, Memb_list* _ml, int _type){
Node *_nd; double _v = 0.0; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
 _nd = _ml->_nodelist[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v=_v;
{
 { error =  state();
 if(error){fprintf(stderr,"at line 42 in file AMPA_Channel_Potentiation.mod:\nBREAKPOINT {\n"); nrn_complain(_p); abort_run(error);}
 }  }}

}

static void terminal(){}

static void _initlists() {
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = P_columnindex;  _dlist1[0] = DP_columnindex;
 _slist1[1] = P_columnindex;  _dlist1[1] = DP_columnindex;
_first = 0;
}

#if NMODL_TEXT
static void register_nmodl_text_and_filename(int mech_type) {
    const char* nmodl_filename = "AMPA_Channel_Potentiation.mod";
    const char* nmodl_file_text = 
  "TITLE AMPA_Channel with Calcium Dependent Potentiation\n"
  "\n"
  "NEURON {\n"
  "    POINT_PROCESS AMPA_Channel_Potentiation\n"
  "    USEION na WRITE ina VALENCE 1\n"
  "    USEION k WRITE ik VALENCE 1\n"
  "    RANGE baseline_gmax, ena, ek, p_ratio, calcium_threshold, potentiation_rate, potentiation_strength, decay_rate, P\n"
  "    POINTER receptor_activation, NMDA_cai\n"
  "}\n"
  "\n"
  "PARAMETER {\n"
  "    receptor_activation (1)\n"
  "    baseline_gmax = 0.1 (uS)   : Baseline maximum conductance\n"
  "    ena = 50 (mV)              : Sodium reversal potential\n"
  "    ek = -90 (mV)              : Potassium reversal potential\n"
  "    p_ratio = 10               : Sodium-to-potassium permeability ratio\n"
  "    calcium_threshold = 0.0001 (mM) : Calcium threshold for potentiation\n"
  "    potentiation_rate = 0.01 (/ms)  : Potentiation rate\n"
  "    decay_rate = 0.005 (/ms)        : Potentiation decay rate\n"
  "    potentiation_strength = 1 (1)  : Multiplier for potentiation\n"
  "}\n"
  "\n"
  "ASSIGNED {\n"
  "    v (mV)                     : Membrane potential\n"
  "    NMDA_cai (mM)              : NMDA calcium concentration\n"
  "    ina (nA)                   : Sodium current\n"
  "    ik (nA)                    : Potassium current\n"
  "    gna (uS)                   : Sodium conductance\n"
  "    gk (uS)                    : Potassium conductance\n"
  "    gmax (uS)                  : Dynamic maximum conductance\n"
  "}\n"
  "\n"
  "STATE {\n"
  "    P (1)                      : Potentiation factor (scaling gmax)\n"
  "}\n"
  "\n"
  "INITIAL {\n"
  "    P = 0                      : No potentiation initially\n"
  "    gmax = baseline_gmax       : Initial gmax\n"
  "}\n"
  "\n"
  "BREAKPOINT {\n"
  "    SOLVE state METHOD cnexp\n"
  "\n"
  "    : Scale gmax by potentiation factor\n"
  "    gmax = baseline_gmax * (1 + P * potentiation_strength)\n"
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
  "    : Potentiation dynamics driven by calcium\n"
  "    if (NMDA_cai > calcium_threshold) {\n"
  "        P' = potentiation_rate * NMDA_cai\n"
  "    } else {\n"
  "        P' = -decay_rate * P               : Potentiation decays to 0 when calcium is low\n"
  "    }\n"
  "}\n"
  ;
    hoc_reg_nmodl_filename(mech_type, nmodl_filename);
    hoc_reg_nmodl_text(mech_type, nmodl_file_text);
}
#endif