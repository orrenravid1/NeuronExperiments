#include <stdio.h>
#include "hocdec.h"
#define IMPORT extern __declspec(dllimport)
IMPORT int nrnmpi_myid, nrn_nobanner_;

extern "C" void _GenericLigand_reg();
extern "C" void _GenericReceptor_reg();
extern "C" void _NMDA_Channel_reg();

extern "C" void modl_reg(){
	//nrn_mswindll_stdio(stdin, stdout, stderr);
    if (!nrn_nobanner_) if (nrnmpi_myid < 1) {
	fprintf(stderr, "Additional mechanisms from files\n");

fprintf(stderr," GenericLigand.mod");
fprintf(stderr," GenericReceptor.mod");
fprintf(stderr," NMDA_Channel.mod");
fprintf(stderr, "\n");
    }
_GenericLigand_reg();
_GenericReceptor_reg();
_NMDA_Channel_reg();
}
