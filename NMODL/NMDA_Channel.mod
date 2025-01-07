TITLE NMDA_Channel
COMMENT
A simple channel mechanism that takes:
 - "activation" from some receptor
 - voltage-dependent Mg2+ block
And produces a nonspecific current i.
ENDCOMMENT

NEURON {
    POINT_PROCESS NMDA_Channel
    NONSPECIFIC_CURRENT i
    RANGE i, gmax, e, mgblock, mg, alpha_mg, beta_mg
    POINTER receptor_activation
}

PARAMETER {
    gmax     = 0.1 (uS)     : Maximum conductance
    e        = 0   (mV)     : Reversal potential
    mg       = 1   (mM)     : [Mg2+] ext
    alpha_mg = 0.062 (/mV)  : Voltage-dependence factor
    beta_mg  = 3.57         : Mg2+ block scaling
}

ASSIGNED {
    v      (mV)
    receptor_activation (1)
    i      (nA)
    mgblock
}

BREAKPOINT {
    : Mg2+ block factor
    mgblock = 1 / (1 + mg*beta_mg*exp(-alpha_mg*v))

    : Current = gmax * activation * mgblock * (V - E)
    i = gmax * receptor_activation * mgblock * (v - e)
}
