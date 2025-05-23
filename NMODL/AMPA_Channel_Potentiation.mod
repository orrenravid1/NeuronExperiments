TITLE AMPA_Channel with Calcium Dependent Potentiation and Depression

NEURON {
    POINT_PROCESS AMPA_Channel_Potentiation
    USEION na WRITE ina VALENCE 1
    USEION k WRITE ik VALENCE 1
    RANGE baseline_gmax, ena, ek, p_ratio
    RANGE potentiation_threshold, depression_threshold, potentiation_rate, potentiation_strength, depression_rate, depression_strength, P, D
    RANGE calcium_baseline, potentiation_decay, depression_decay
    POINTER receptor_activation, local_cai
}

PARAMETER {
    receptor_activation (1)
    baseline_gmax = 0.1 (uS)   : Baseline maximum conductance
    ena = 50 (mV)              : Sodium reversal potential
    ek = -90 (mV)              : Potassium reversal potential
    p_ratio = 10               : Sodium-to-potassium permeability ratio
    potentiation_threshold = 0.5 (mM) : Calcium threshold for potentiation
    depression_threshold = 0.5 (mM) : Calcium threshold for depression
    potentiation_rate = 0.01 (/ms)  : Potentiation rate
    depression_rate = 0.005 (/ms)   : Depression rate
    potentiation_strength = 1 (1)  : Multiplier for potentiation
    depression_strength = 1 (1) : Multiplier for depression
    calcium_baseline = 0.0001 (mM)
    potentiation_decay = 0.1 (1)
    depression_decay = 0.1 (1)
}

ASSIGNED {
    v (mV)                     : Membrane potential
    local_cai (mM)             : Local calcium concentration
    ina (nA)                   : Sodium current
    ik (nA)                    : Potassium current
    gna (uS)                   : Sodium conductance
    gk (uS)                    : Potassium conductance
    gmax (uS)                  : Dynamic maximum conductance
}

STATE {
    P (1)                      : Potentiation factor (scaling gmax)
    D (1)                      : Depression factor (scaling gmax)
}

INITIAL {
    P = 0                     : No potentiation initially
    D = 0                     : No depression initially
    gmax = baseline_gmax       : Initial gmax
}

BREAKPOINT {
    SOLVE state METHOD cnexp

    : Scale gmax by potentiation factor
    gmax = max(0, baseline_gmax * (1 + P * potentiation_strength - D * depression_strength))

    : Calculate sodium and potassium conductances using permeability ratio
    gna = gmax * receptor_activation * p_ratio / (1 + p_ratio)
    gk = gmax * receptor_activation / (1 + p_ratio)

    : Calculate sodium and potassium currents
    ina = gna * (v - ena)
    ik = gk * (v - ek)
}

DERIVATIVE state {
    LOCAL relative_calcium

    : Compute relative calcium
    relative_calcium = local_cai / calcium_baseline

    : Potentiation dynamics (smooth thresholding for calcium)
    P' = potentiation_rate * max(0, relative_calcium - potentiation_threshold) * (1 - P)
       - potentiation_decay * P

    : Depression dynamics (smooth thresholding for calcium)
    : Only unrealistic thing here is that depression can happen at 0 calcium
    D' = depression_rate * max(0, depression_threshold - relative_calcium) * (1 - D)
       - depression_decay * D
}

FUNCTION max(x1, x2) {
    if (x1 >= x2){
        max = x1
    }
    else{
        max = x2
    }
}