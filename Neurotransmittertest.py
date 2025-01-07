from neuron import h, load_mechanisms
import matplotlib.pyplot as plt
import numpy as np

h.load_file('stdrun.hoc')

# Create section
soma = h.Section(name='soma')
soma.L = 20
soma.diam = 20

# Create ligand and receptor
ligand = h.GenericLigand(soma(0.5))
ligand.C_init = 0.1

receptor = h.GenericReceptor(soma(0.5))
receptor.capacity = 1.0
receptor.n_ligands = 1
receptor.kd1 = 100
receptor.efficacy1 = 0.3
receptor.decay1 = 0.2

# Use _ref assignment instead of setpointer
receptor._ref_C_lig1 = ligand._ref_C

# Stimulus for ligand release
stim = h.NetStim()
stim.start = 10              # Start stimulus at 10 ms
stim.number = 5              # Fire 5 spikes
stim.interval = 10           # 10 ms between spikes

# Connect stimulus to ligand via NetCon (to trigger spikes)
nc1 = h.NetCon(stim, ligand)
nc1.weight[0] = 0.2          # Amount of ligand release

# Record data
t = h.Vector().record(h._ref_t)
ligand_conc = h.Vector().record(ligand._ref_C)
receptor_act = h.Vector().record(receptor._ref_activation)

# Confirm POINTER assignment
print("Ligand C before init:", ligand.C)
print("Receptor C_lig1 before init:", receptor.C_lig1)

# Initialize
h.finitialize(-65)

print("Ligand C after init:", ligand.C)
print("Receptor C_lig1 after init:", receptor.C_lig1)

# Run simulation
h.continuerun(100)

print("Ligand C after run:", ligand.C)
print("Receptor Activation after run:", receptor.activation)

# Plot Results
plt.figure(figsize=(10, 6))

plt.subplot(3, 1, 1)
plt.plot(t, ligand_conc, label="Ligand Concentration (uM)")
plt.ylabel("Concentration (uM)")
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(t, receptor_act, label="Receptor Activation")
plt.ylabel("Activation")
plt.legend()

plt.tight_layout()
plt.show()
