%---------------------------------------------------------------------------------------------------
% For Paper
% "Distributed Flocking Control with Ellipsoidal Level Sets"
% by P. Hastedt, A. Datar, K. Kocev, and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------

function [T, T_normalized, normalizationFactor] = calculateEllipsoidalTransformation(semiAxes, rotation)
%CALCULATEELLIPSOIDALTRANSFORMATION
% Calculate the ellipsoidal transformation matrix
% Inputs:
%   semiAxes:   vector of ellipse semi-axes
%   rotation:   vector of rotation angles (rad)
% Outputs:
%   T:                      transformation matrix
%   T_normalized:           normalized transformation matrix
%   normalizationFactor:    normalizationFactor

dimension = length(semiAxes);
if ~((dimension==1 && isempty(rotation)) || (dimension==2 && length(rotation) == 1) ||(dimension==3 && length(rotation) == 3))
    error('Dimensions do not match.')
end

normalizationFactor = 1/nthroot(det(diag(1./semiAxes)),dimension);
if dimension == 2 
   R = [cos(rotation(1)) -sin(rotation(1));
        sin(rotation(1))  cos(rotation(1))];
elseif dimension == 3
   rotRad = rotation*180/pi;
   R = rotx(rotRad(1))*roty(rotRad(2))*rotz(rotRad(3));
else
    R = 1;
end
Lambda = diag(semiAxes);
T = inv(R*Lambda);
T_normalized = T*normalizationFactor;
end