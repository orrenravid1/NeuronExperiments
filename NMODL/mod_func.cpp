#include <stdio.h>
#include "hocdec.h"
#define IMPORT extern __declspec(dllimport)
IMPORT int nrnmpi_myid, nrn_nobanner_;

extern "C" void _AMPA_Channel_reg();
extern "C" void _AMPA_Channel_Potentiation_reg();
extern "C" void _GABA_A_Channel_reg();
extern "C" void _GenericLigand_reg();
extern "C" void _GenericReceptor_reg();
extern "C" void _NMDA_Channel_reg();
extern "C" void _NMDA_Channel_Calcium_reg();

extern "C" void modl_reg(){
	//nrn_mswindll_stdio(stdin, stdout, stderr);
    if (!nrn_nobanner_) if (nrnmpi_myid < 1) {
	fprintf(stderr, "Additional mechanisms from files\n");

fprintf(stderr," AMPA_Channel.mod");
fprintf(stderr," AMPA_Channel_Potentiation.mod");
fprintf(stderr," GABA_A_Channel.mod");
fprintf(stderr," GenericLigand.mod");
fprintf(stderr," GenericReceptor.mod");
fprintf(stderr," NMDA_Channel.mod");
fprintf(stderr," NMDA_Channel_Calcium.mod");
fprintf(stderr, "\n");
    }
_AMPA_Channel_reg();
_AMPA_Channel_Potentiation_reg();
_GABA_A_Channel_reg();
_GenericLigand_reg();
_GenericReceptor_reg();
_NMDA_Channel_reg();
_NMDA_Channel_Calcium_reg();
}
