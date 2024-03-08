%---------------------------------------------------------------------------------------------------
% For Paper
% "Distributed Flocking Control with Ellipsoidal Level Sets"
% by P. Hastedt, A. Datar, K. Kocev, and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------

clear;
run contents.m

% Algorithms
% 1: Proposed Elliptic Flocking
% 2: Elliptic Wang Flocking

% The desired algorithm to generate a configuration file for can be
% selected via the algorithmIndex

algorithmIndex = 1;

configName = "config"; % name of created configuration file

createConfig(algorithms, algorithmIndex, configName)

function cfg = createConfig(algorithms, algorithmIndex, configName)
% set parameters
switch(algorithmIndex)

    % 1: Proposed Elliptic Flocking
    case (1)
        cfg.d = 7;                  % desired inter-agent distance
        cfg.m = 2;                  % space dimension
        cfg.ellipseAxes = [1 2/3];  % ellipse semi-axis lengths relative to desired distance
        cfg.ellipseRotation = [0];  % ellipse rotation in rad
        cfg.h = 0.2;                % bump-function parameter
        cfg.s = 0.1;                % sigma norm parameter 
        cfg.c_alpha = 5;            % alpha flocking parameter
        cfg.c_gamma = 0.5;          % gamma flocking parameter

    % 2: Elliptic Wang Flocking
    case (2)
        cfg.d = 7;                  % desired inter-agent distance
        cfg.m = 2;                  % space dimension
        cfg.ellipseAxes = [1 2/3];  % ellipse semi-axis lengths relative to desired distance
        cfg.ellipseRotation = 0;    % ellipse rotation in rad
        cfg.h = 0.8;                % bump function parameter  
        cfg.tau = 0.3;              % bump function parameter  
        cfg.epsilon = 0.1;          % sigma norm parameter     
        cfg.c_alpha = 5;            % alpha flocking parameter
        cfg.c_alpha_phi = 1;        % alpha flocking parameter
        cfg.c_gamma = 0.5;          % gamma flocking parameter
end

save(strcat(algorithms(algorithmIndex),'/cfg/',configName,'.mat'),'cfg');
end

