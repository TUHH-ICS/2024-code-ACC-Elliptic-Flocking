%---------------------------------------------------------------------------------------------------
% For Paper
% "Distributed Flocking Control with Ellipsoidal Level Sets"
% by P. Hastedt, A. Datar, K. Kocev, and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------

function setup = generateSetupEllipticWang(cfg, param, init)
setup = struct;
Network = IdealNetwork(param.agentCount, param.dT, param.dimension, param.range);
setup.Network = Network;
Agents = cell(param.agentCount, 1);

for i = 1:length(Agents)
    Agents{i} = EllipticWangAgent(Network.getId(), param, init.pos(:,i), init.vel(:,i), cfg);
end
setup.Agents = [Agents{:}];
end