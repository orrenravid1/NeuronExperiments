TITLE AMPA_Channel
COMMENT
AMPA receptor mechanism with sodium-potassium permeability ratio.
ENDCOMMENT

NEURON {
    POINT_PROCESS AMPA_Channel
    USEION na WRITE ina
    USEION k WRITE ik
    RANGE gmax, ena, ek, p_ratio
    POINTER receptor_activation
}

PARAMETER {
    gmax = 0.1 (uS)    : Maximum conductance
    ena  = 50  (mV)    : Sodium reversal potential
    ek   = -90 (mV)    : Potassium reversal potential
    p_ratio = 10       : Sodium-to-potassium permeability ratio
}

ASSIGNED {
    v      (mV)        : Membrane potential
    receptor_activation (1)
    ina    (nA)        : Sodium current
    ik     (nA)        : Potassium current
    gna    (uS)        : Effective sodium conductance
    gk     (uS)        : Effective potassium conductance
}

BREAKPOINT {
    : Adjust conductance based on permeability ratio
    gna = gmax * receptor_activation * p_ratio / (1 + p_ratio)
    gk  = gmax * receptor_activation / (1 + p_ratio)

    : Currents
    ina = gna * (v - ena)
    ik  = gk * (v - ek)
}
