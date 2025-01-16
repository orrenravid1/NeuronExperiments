TITLE General Receptor with Per-Ligand Decay and Flexible Targeting

NEURON {
    POINT_PROCESS GenericReceptor
    RANGE baseline_activity
    RANGE activation, capacity, occupancy
    RANGE n_ligands

    : Parameters per ligand
    RANGE kd1, efficacy1, bound1, decay1
    RANGE kd2, efficacy2, bound2, decay2
    RANGE kd3, efficacy3, bound3, decay3
    RANGE kd4, efficacy4, bound4, decay4

    POINTER C_lig1, C_lig2, C_lig3, C_lig4
}

PARAMETER {
    baseline_activity = 0

    n_ligands = 1         : Number of active ligands (1-4)

    capacity = 1.0        : Max binding capacity (fractional, e.g., 1 = 100%)

    : Ligand 1
    kd1 = 0.5 (uM)
    efficacy1 = 1.0
    decay1 = 0.1 (/ms)

    : Ligand 2
    kd2 = 1.0 (uM)
    efficacy2 = 0.5
    decay2 = 0.05 (/ms)

    : Ligand 3
    kd3 = 0.7 (uM)
    efficacy3 = -1.0
    decay3 = 0.2 (/ms)

    : Ligand 4
    kd4 = 0.3 (uM)
    efficacy4 = 0.0
    decay4 = 0.01 (/ms)
}

ASSIGNED {
    C_lig1 (uM)    : POINTER for Ligand 1
    C_lig2 (uM)    : POINTER for Ligand 2
    C_lig3 (uM)    : POINTER for Ligand 3
    C_lig4 (uM)    : POINTER for Ligand 4
    activation (1)  : Total activation level
    occupancy (1)   : Total occupancy
}

STATE {
    bound1 (1)      : Bound state for ligand 1
    bound2 (1)      : Bound state for ligand 2
    bound3 (1)      : Bound state for ligand 3
    bound4 (1)      : Bound state for ligand 4
}

INITIAL {
    bound1 = 0
    bound2 = 0
    bound3 = 0
    bound4 = 0
    activation = 0
    occupancy = 0
}

BREAKPOINT {
    LOCAL net_activation
    SOLVE states METHOD cnexp
    occupancy = bound1 + bound2 + bound3 + bound4
    net_activation = bound1 * efficacy1 + bound2 * efficacy2 + bound3 * efficacy3 + bound4 * efficacy4
    activation = min(1, max(0, baseline_activity + net_activation))
}

DERIVATIVE states {
    LOCAL occ1, occ2, occ3, occ4, total_demand, remaining_capacity, scale_factor

    : Compute "ideal" contributions based on ligand affinity
    if (n_ligands >= 1) {
        occ1 = occ(C_lig1, kd1)
    } else {
        occ1 = 0
    }
    if (n_ligands >= 2) {
        occ2 = occ(C_lig2, kd2)
    } else {
        occ2 = 0
    }
    if (n_ligands >= 3) {
        occ3 = occ(C_lig3, kd3)
    } else {
        occ3 = 0
    }
    if (n_ligands >= 4) {
        occ4 = occ(C_lig4, kd4)
    } else {
        occ4 = 0
    }

    : Sum total demand
    total_demand = occ1 + occ2 + occ3 + occ4

    remaining_capacity = capacity - occupancy

    : Scale binding if demand exceeds capacity
    if (total_demand > remaining_capacity && total_demand > 0) {
        scale_factor = remaining_capacity / total_demand
    }
    else{
        scale_factor = 1.0
    }

    : Apply scaled contributions
    if (n_ligands >= 1) {
        bound1' = scale_factor * occ1 - bound1 * decay1
    }
    if (n_ligands >= 2) {
        bound2' = scale_factor * occ2 - bound2 * decay2
    }
    if (n_ligands >= 3) {
        bound3' = scale_factor * occ3 - bound3 * decay3
    }
    if (n_ligands >= 4) {
        bound4' = scale_factor * occ4 - bound4 * decay4
    }
}

FUNCTION occ(C_lig(uM), kd(uM)) {
    if (C_lig + kd <= 0) {
        occ = 0
    } else {
        occ = C_lig / (kd + C_lig)
    }
}

FUNCTION min(x1, x2) {
    if (x1 <= x2){
        min = x1
    }
    else{
        min = x2
    }
}

FUNCTION max(x1, x2) {
    if (x1 >= x2){
        max = x1
    }
    else{
        max = x2
    }
}