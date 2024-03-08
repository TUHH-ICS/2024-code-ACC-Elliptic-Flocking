%---------------------------------------------------------------------------------------------------
% For Paper
% "Distributed Flocking Control with Ellipsoidal Level Sets"
% by P. Hastedt, A. Datar, K. Kocev, and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------

clear; close all;

% load contents and paths
addpath(genpath('mas-simulation/lib'), genpath('lib'), genpath('data'), genpath('src'))
run contents.m

initPath = "data/init/";
outPath = "data/out/";

%% Simulation Setup
% The desired combination of algorithm and scenario can be selected via the
% algorithmIndex and scenarioIndex variables.

% Algorithms (to be selected via algorithmIndex)
% 1: Proposed Elliptic Flocking
% 2: Elliptic Wang Flocking

% Scenarios (to be selected via scenarioIndex)
% 1: Scenario 1
% 2: Scenario 2
% 3: Scenario 3 - Small Eccentricity
% 4: Scenario 3 - Large Eccentricity
% 5: Custom Scenario (custom configuration files can be generated with the 
%    /lib/generateConfig.m script)

algorithmIndex = 1;
scenarioIndex = 1;

% Simulation Parameters
if scenarioIndex == 1 % Scenario 1
    configFile = "scenario_1.mat";
    initFile = "initialStates_20.mat";
    Tf               = 50;      % Simulation duration [s]
    param.agentCount = 20;      % Number of agents in the network
    param.dimension  = 2;       % Dimension of the space the agents move in
    param.dT         = 0.004;   % Size of the simulation time steps [s]
    param.range      = 8.4;     % Agent interaction range
    param.reference = zeros(4,1);% gamma agent

elseif scenarioIndex == 2 % Scenario 2
    configFile = "scenario_2.mat";
    initFile = "initialStates_20.mat";
    Tf               = 100;     % Simulation duration [s]
    param.agentCount = 20;      % Number of agents in the network
    param.dimension  = 2;       % Dimension of the space the agents move in
    param.dT         = 0.002;   % Size of the simulation time steps [s]
    param.range      = 8.4;     % Agent interaction range
    param.reference = zeros(4,1);% gamma agent

elseif scenarioIndex == 3 % Scenario 3 - Small Eccentricity
    configFile = "scenario_3_1.mat";
    initFile = "initialStates_20_3D.mat";
    Tf               = 80;      % Simulation duration [s]
    param.agentCount = 20;      % Number of agents in the network
    param.dimension  = 3;       % Dimension of the space the agents move in
    param.dT         = 0.002;   % Size of the simulation time steps [s]
    param.range      = 8.4;     % Agent interaction range
    param.reference = zeros(6,1);% gamma agent

elseif scenarioIndex == 4 % Scenario 3 - Large Eccentricity
    configFile = "scenario_3_2.mat";
    initFile = "initialStates_20_3D.mat";
    Tf               = 80;     % Simulation duration [s]
    param.agentCount = 20;      % Number of agents in the network
    param.dimension  = 3;       % Dimension of the space the agents move in
    param.dT         = 0.002;   % Size of the simulation time steps [s]
    param.range      = 8.4;     % Agent interaction range
    param.reference = zeros(6,1);% gamma agent

else % Custom Scenario
    % enter parameters for custom scenario here
end

init = load(initPath+initFile);
outFileName = generateOutFileName(configFile);
cfg = load(strcat(algorithms(algorithmIndex),"/cfg/")+configFile).cfg;
setup = generateSetup{algorithmIndex}(cfg, param, init);

%% Run Simulation
sim = SimulationManager(setup.Network, setup.Agents);
leech = preallocate{algorithmIndex}(setup, sim, Tf);
leechOut = performSimulation(sim, leech, Tf);

% convert DataLeech to struct
out.data = leechOut.data;
out.t = leechOut.t;

% save data
save(strcat(outPath,erase(algorithms(algorithmIndex),srcPath),"/")+outFileName, 'out', 'setup', 'param', 'cfg', 'init');