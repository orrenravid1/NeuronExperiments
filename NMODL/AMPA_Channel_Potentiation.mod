TITLE AMPA_Channel with Calcium Dependent Potentiation

NEURON {
    POINT_PROCESS AMPA_Channel_Potentiation
    USEION na WRITE ina VALENCE 1
    USEION k WRITE ik VALENCE 1
    RANGE baseline_gmax, ena, ek, p_ratio, calcium_threshold, potentiation_rate, potentiation_strength, decay_rate, P
    POINTER receptor_activation, NMDA_cai
}

PARAMETER {
    receptor_activation (1)
    baseline_gmax = 0.1 (uS)   : Baseline maximum conductance
    ena = 50 (mV)              : Sodium reversal potential
    ek = -90 (mV)              : Potassium reversal potential
    p_ratio = 10               : Sodium-to-potassium permeability ratio
    calcium_threshold = 0.0001 (mM) : Calcium threshold for potentiation
    potentiation_rate = 0.01 (/ms)  : Potentiation rate
    decay_rate = 0.005 (/ms)        : Potentiation decay rate
    potentiation_strength = 1 (1)  : Multiplier for potentiation
}

ASSIGNED {
    v (mV)                     : Membrane potential
    NMDA_cai (mM)              : NMDA calcium concentration
    ina (nA)                   : Sodium current
    ik (nA)                    : Potassium current
    gna (uS)                   : Sodium conductance
    gk (uS)                    : Potassium conductance
    gmax (uS)                  : Dynamic maximum conductance
}

STATE {
    P (1)                      : Potentiation factor (scaling gmax)
}

INITIAL {
    P = 0                      : No potentiation initially
    gmax = baseline_gmax       : Initial gmax
}

BREAKPOINT {
    SOLVE state METHOD cnexp

    : Scale gmax by potentiation factor
    gmax = baseline_gmax * (1 + P * potentiation_strength)

    : Calculate sodium and potassium conductances using permeability ratio
    gna = gmax * receptor_activation * p_ratio / (1 + p_ratio)
    gk = gmax * receptor_activation / (1 + p_ratio)

    : Calculate sodium and potassium currents
    ina = gna * (v - ena)
    ik = gk * (v - ek)
}

DERIVATIVE state {
    : Potentiation dynamics driven by calcium
    if (NMDA_cai > calcium_threshold) {
        P' = potentiation_rate * NMDA_cai
    } else {
        P' = -decay_rate * P               : Potentiation decays to 0 when calcium is low
    }
}
