%---------------------------------------------------------------------------------------------------
% For Paper
% "Distributed Flocking Control with Ellipsoidal Level Sets"
% by P. Hastedt, A. Datar, K. Kocev, and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------

% Available algorithms and corresponding functions

% Algorithms
% 1: Proposed Elliptic Flocking
% 2: Elliptic Wang Flocking

srcPath = "src/";

algorithms = srcPath + [...
    "elliptic_T_flocking"
    "elliptic_wang_flocking"
    ];

preallocate = {...
    @preallocateEllipticT
    @preallocateEllipticWang
    };

generateSetup = {...
    @generateSetupEllipticT
    @generateSetupEllipticWang
    };

