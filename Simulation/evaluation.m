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
addpath(genpath('lib'), genpath('data'))

%% Data Selection
dataPath = "data/evaluation/";

% Available Data
simData = [
    "Proposed_Scenario_1"    % 1
    "Proposed_Scenario_2"    % 2
    "Wang_Scenario_1"        % 3
    "Wang_Scenario_2"        % 4
    "Proposed_Scenario_3_1"  % 5
    "Proposed_Scenario_3_2"  % 6
    ];

% Scenarios can be compared by adding the indices of the desired data sets
% in the simData array to the dataSelection array. To reproduce the results
% from the paper, set the scenarioIndex to 1, 2, or 3 for the corresponding
% scenario.

% Scenario Comparison
% 1: Scenario 1
% 2: Scenario 2
% 3: Scenario 3
% 4: Custom comparison

evaluationIndex = 1;

if evaluationIndex == 1 % Scenario 1 Comparison
    dataSelection = [1,3];
elseif evaluationIndex == 2 % Scenario 2 Comparison
    dataSelection = [2,4];
elseif evaluationIndex == 3 % Scenario 3 Comparison
    dataSelection = [5,6];
else % Custom Comparison
    % insert indices for custom comparison here
    dataSeletion = [];
end

%% Comparison/Evaluation
% get names of data sets
for i = 1:length(dataSelection)
    names{i} = replace(erase(simData(dataSelection(i)),".mat"),"_","\_");
end

% plot final states with desired ellipse
if evaluationIndex ~= 3
    for i=1:length(dataSelection)
        plotFinalState(simData(dataSelection(i)), dataPath)
    end
end

% comparison of final irregularity and convergence time
finalIrregularity = zeros(1,length(dataSelection));
convergenceTime = zeros(1,length(dataSelection));
for i=1:length(dataSelection)
    load(dataPath+simData(dataSelection(i)))
    jpos = calculatePerformanceIndices(out,cfg, 8.4, 7, 0);
    figure()
    plot(out.t,jpos);
    xlabel('time');
    ylabel('J');
    title("Lattice Irregularity "+names{i});
    finalIrregularity(i) = jpos(end);
    convergenceTime(i) = out.t(find(abs(jpos-finalIrregularity(i))>0.02,1,'last'));
end

figure()
bar(1:i,finalIrregularity);
set(gca, 'XTick', 1:length(names),'XTickLabel',names);
title('Final Lattice Irregularity')

figure()
bar(1:i,convergenceTime);
set(gca, 'XTick', 1:length(names),'XTickLabel',names);
title('Convergence Time')

% input rms comparison
u_rms = zeros(1,length(dataSelection));
for i = 1:length(dataSelection)
    load(dataPath+simData(dataSelection(i)))
    u_rms(i) = calculateTotalInputUsed(out);
end
figure()
bar(1:i,u_rms);
set(gca, 'XTick', 1:length(names),'XTickLabel',names);
title('RMS Input Values')

% trajectory visualization for 3D scenarios
if evaluationIndex == 3
for i = 1:length(dataSelection)
    load(dataPath+simData(dataSelection(i)))
    pos = out.data.position;
    figure()
    for j = 1:size(pos,3)
        % trajectories
        plot3(pos(:,1,j),pos(:,2,j),pos(:,3,j),'black'); hold on;
        % initial positions
        plot3(pos(1,1,j),pos(1,2,j),pos(1,3,j),'bo'); hold on;
        % final positions
        plot3(pos(end,1,j),pos(end,2,j),pos(end,3,j),'bx'); hold on;
        grid on;
    end 
end
end

