%---------------------------------------------------------------------------------------------------
% For Paper
% "Distributed Flocking Control with Ellipsoidal Level Sets"
% by P. Hastedt, A. Datar, K. Kocev, and H. Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Philipp Hastedt
%---------------------------------------------------------------------------------------------------

function plotFinalState(data, dataPath)
figure()
name = replace(erase(data,".mat"),"_","\_");
load(dataPath+data);

plot(squeeze(out.data.position(end,1,:)),squeeze(out.data.position(end,2,:)),'bx','MarkerSize',10,'LineWidth',1); hold on;

% find agent closest to origin
N = size(out.data.position,3);
distanceFromOrigin = inf*ones(1,N);
for i = 1:N
    distanceFromOrigin(i) = norm(out.data.position(end,:,i));
end
[~,idx] = min(distanceFromOrigin);

% draw ellipsis
T = calculateEllipsoidalTransformation(cfg.ellipseAxes,cfg.ellipseRotation);
th = 0:pi/50:2*pi;
x = out.data.position(end,1,idx);
y = out.data.position(end,2,idx);
ellipse_distance = inv(T)*[cfg.d * cos(th);cfg.d * sin(th)] + [x;y];
plot(ellipse_distance(1,:),ellipse_distance(2,:),'k--','LineWidth',0.25); hold on;

axis equal
title("Agent Trajectories "+ name);
xlabel('x');
ylabel('y');
end

