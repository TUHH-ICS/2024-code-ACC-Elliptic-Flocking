%---------------------------------------------------------------------------------------------------
% For Paper
% "Distributed Flocking Control with Ellipsoidal Level Sets"
% by P. Hastedt, A. Datar, K. Kocev, and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------

function [Jq] = calculatePerformanceIndices(data, cfg, rc, d, plotResults, varargin)
agentCount = length(data.data.position(1,1,:));
Jq = zeros(1,length(data.t));
Jp = zeros(1,length(data.t));
[~, T_normalized, normalizationFactor] = calculateEllipsoidalTransformation(cfg.ellipseAxes,cfg.ellipseRotation);
for t = 1:length(data.t)
    position = data.data.position;

    % calculate position performance index
    countQ = 0;
    for i  = 1:agentCount
        for j = 1:agentCount
            qij = norm(T_normalized*(position(t,:,i)-position(t,:,j))');
            if (qij<=rc*normalizationFactor) && (i~=j)
                Jq(t) = Jq(t)+(qij-d*normalizationFactor)^2;
                countQ = countQ+1;
            end
        end
    end
    if countQ ~=0
        Jq(t) = Jq(t)/countQ;
    end
end
if plotResults
    figure()
    plot(data.t, Jq);
    xlabel('time');
    ylabel('J_q');
    title('Position Irregularity');
    grid on;
    set(gca, 'XLim', [0 data.t(end-1)]);
end

end