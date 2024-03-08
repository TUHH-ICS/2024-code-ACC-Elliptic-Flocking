%---------------------------------------------------------------------------------------------------
% For Paper
% "Distributed Flocking Control with Ellipsoidal Level Sets"
% by P. Hastedt, A. Datar, K. Kocev, and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------

function leech = preallocateEllipticT(setup, sim, Tf)
    % Estimate the number of steps based on the final simulation time
    % Preallocate storage for simulation results
    steps = sim.estimateSteps(Tf);
    leech = DataLeech(setup.Agents, steps, 'position', 'velocity', 'u', 'num_N', 'id', 'neighbors');
end