%---------------------------------------------------------------------------------------------------
% For Paper
% "Distributed Flocking Control with Ellipsoidal Level Sets"
% by P. Hastedt, A. Datar, K. Kocev, and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------

function u_rms = calculateTotalInputUsed(out)
data = out.data;
agentCount = size(data.u,3);
u_rms = 0;
for i = 1:agentCount
    u_rms = u_rms + rms(data.u(:,:,i),'all');
end
end