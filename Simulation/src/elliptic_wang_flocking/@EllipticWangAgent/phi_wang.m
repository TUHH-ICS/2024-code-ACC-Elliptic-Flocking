%---------------------------------------------------------------------------------------------------
% For Paper
% "Distributed Flocking Control with Ellipsoidal Level Sets"
% by P. Hastedt, A. Datar, K. Kocev, and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------

function alpha = phi_wang(obj, qij_sigma, eij)
phi   = obj.c_alpha*tanh(obj.c_alpha_phi*eij);
alpha = obj.s_h(qij_sigma/obj.r).* phi;
end
