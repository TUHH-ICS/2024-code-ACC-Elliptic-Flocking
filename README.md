# 2023-code-Distributed Flocking Control with Ellipsoidal Level Sets

## General

This repository the code for the paper:

> P. Hastedt, A. Datar,  K. Kocev, and H. Werner, "Distributed Flocking Control with Ellipsoidal Level Sets"

This code may be used to recreate and validate the simulation results and figures from the paper. 

The code in this repository was tested in the following environment:

* *Windows 10* Version 22H2
* *Matlab* 2023a

## Level Curve Comparison

The directory `Level Set Comparison` contains code for reproducing the the level set comparison figure from the paper (Figure 2).

## Simulation Scenarios
The directory `Simulation` contains the code for reproducing the simulation results and evaluations from the paper. The simulations are obtained using the open-source Matlab simulation library WiMAS.

> C. Hespe, A. Datar, D. Schneider, H. Saadabad, H. Werner,  H. Frey (2023). Distributed H 2 Controller Synthesis for Multi-Agent Systems with Stochastic Packet Loss. 2023 American Control Conference (ACC). 4197-4202
https://doi.org/10.23919/ACC55779.2023.10156194

This code furthermore contains an implementation of the flocking algorithm of the paper
> F. Wang, G.Wang, Y. Chen (2022). Adaptive Spacing Policy Design of Flocking Control for Multi-agent Vehicular Systems. IFAC PapersOnLine. vol. 55. no. 37. pp. 524-529.
https://doi.org/10.1016/j.ifacol.2022.11.236

#### Setup
When downloading the code from Zenodo, the MAS-simulation submodule directory `Simulation/mas-simulation` will be empty. This can be resolved by either directly downloading the code for the paper from GitHub or by copying the source code of the [WiMAS library](https://github.com/TUHH-ICS/MAS-Simulation) to the corresponding directory.

#### Simulation
In the script `simulation.m`, the desired algorithms and scenarios can be selected via the `algorithmIndex` and `scenarioIndex` variables. It is also possible to generate custom simulation scenarios. The simulation results will be saved in the `Simulation/out` directory and can then be used for evaluation. 

#### Evaluation
The figures in the paper can be reproduced with the `evaluation.m`script. The scenario can be selected via the `evaluaitonIndex`variable. In order to add additional data to the evaluation, copy the data set to be evaluated in the `Simulation/out/evaluation` directory and add the name of the data set to the `simData` array in the evaluation script.