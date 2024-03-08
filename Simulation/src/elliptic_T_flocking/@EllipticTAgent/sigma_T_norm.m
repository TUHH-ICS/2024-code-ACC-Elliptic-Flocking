%---------------------------------------------------------------------------------------------------
% For Paper
% "Distributed Flocking Control with Ellipsoidal Level Sets"
% by P. Hastedt, A. Datar, K. Kocev, and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------

function [y, gradient] = sigma_T_norm(obj, z, T)
normed   = sqrt(1 + obj.s * norm(T*z).^2);
y        = (normed - 1) / obj.s;
gradient = (T'*T)*z ./ normed;
end
