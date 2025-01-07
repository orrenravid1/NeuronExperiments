TITLE Single-Ligand Generic Receptor with Decay

NEURON {
    POINT_PROCESS GenericReceptor
    RANGE activation, capacity, occupancy
    
    : Single ligand parameters
    RANGE kd1, efficacy1, bound1, decay1, tmp, tmp1, tmp2
    
    POINTER C_lig1
}

PARAMETER {
    : Binding capacity (maximum fraction that can be bound1)
    capacity = 1.0 (1)
    
    : Ligand binding parameters
    kd1 = 0.5 (uM)        : Dissociation constant
    efficacy1 = 1.0 (1)   : Scaling factor for activation
    decay1 = 0.1 (/ms)    : Unbinding rate
    tmp = 0 (1)
    tmp1 = 0 (1)
    tmp2 = 0 (1)
}

ASSIGNED {
    C_lig1 (uM)           : POINTER to ligand concentration
    activation (1)       : Total activation level (computed from bound1 state)
    occupancy (1)        : Total occupancy (same as bound1 for single ligand)
}

STATE {
    bound1 (1)            : Fraction of receptors bound1 to ligand
}

INITIAL {
    bound1 = 0
    activation = 0
    occupancy = 0
}

BREAKPOINT {
    SOLVE states METHOD cnexp
    
    : Update computed values
    occupancy = bound1
    activation = bound1 * efficacy1
    
    : Debug prints
    VERBATIM
    printf("Time: %.3f, bound1: %.3f, activation: %.3f\n", t, bound1, activation);
    ENDVERBATIM
}

DERIVATIVE states {
    LOCAL occ_equilibrium, total_demand, remaining_capacity, scale_factor
    
    : Calculate equilibrium occupancy based on concentration and Kd
    occ_equilibrium = occ(C_lig1, kd1)
    
    : For single ligand, total_demand is just the equilibrium occupancy
    total_demand = occ_equilibrium
    
    : Calculate remaining binding capacity
    remaining_capacity = capacity - occupancy
    
    : Scale binding if demand exceeds capacity
    if (total_demand > remaining_capacity && total_demand > 0) {
        scale_factor = remaining_capacity / total_demand
    }
    else{
        scale_factor = 1.0
    }
    
    : Solve binding differential equation
    bound1' = scale_factor * occ_equilibrium - bound1 * decay1

    tmp = total_demand
    tmp1 = scale_factor
    tmp2 = remaining_capacity

    VERBATIM
    printf("total_demand: %.3f, scale_factor: %.3f, remaining_capacity: %.3f\n", tmp, tmp1, tmp2);
    ENDVERBATIM
}

FUNCTION occ(C(uM), kd1(uM)) {
    if (C + kd1 <= 0) {
        occ = 0
    } else {
        occ = C / (C + kd1)
    }
}