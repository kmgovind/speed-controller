clear;clc; close all;
% Driver Code

filenametransect = 'boatMotionTransect.gif';
global currentTime
wpThresh = 0.05;

%% Transect implementation
%Initialize Environment and Vehicles
domaintransect = Environment; % Mission Domain
boattransect = Vehicle(33.5,-76.1); % initialize boattransect at start lat/long
boattransect.latitude = boattransect.latitude + km2deg(5);
boattransect.longitude = boattransect.longitude + km2deg(5);

% Track boattransect path
latListTransect = nan(1,domaintransect.endTime - domaintransect.startTime);
longListTransect = nan(1,domaintransect.endTime - domaintransect.startTime);

% Direction Leg used to determine part of transect strategy
% 0 - right; 1 - up; 2 - left; 3 - up
legCount = 0;
[goal.lat, goal.long] = transectWaypoint(legCount, boattransect, domaintransect);

for currentTime = domaintransect.startTime:minutes(domaintransect.timeStep):domaintransect.endTime

    % Find new goal if sufficiently close to previous waypoint
    if deg2nm(distance('gc',goal.lat, goal.long, boattransect.latitude, boattransect.longitude)) <= wpThresh
        legCount = mod(legCount + 1, 4);
        [goal.lat, goal.long] = transectWaypoint(legCount, boattransect, domaintransect);
    end
    
    % Move boattransect towards goal
    boattransect = boattransect.moveBoat(domaintransect, goal.lat, goal.long, currentTime);
    %     fprintf('\nTime: %d\tLat: %d\tLong:%d', currentTime-domaintransect.startTime+1, boattransect.latitude, boattransect.longitude);
    latListTransect(currentTime - domaintransect.startTime + 1) = boattransect.latitude;
    longListTransect(currentTime - domaintransect.startTime + 1) = boattransect.longitude;
   
    % PLOT UPDATE
    hold on;
    
    % plot boattransect location, goal location, and path traveled
    plot(goal.long, goal.lat, 'r*', 'MarkerSize', 20) % Path goal
    plot(boattransect.longitude, boattransect.latitude, 'md', 'MarkerSize', 10) % Current vehicle location
    plot(longListTransect, latListTransect, 'r-', 'LineWidth', 2); % Path to follow
    % set(gca, 'ydir', 'reverse')
    axis([-76.1,-75.1,33.5,34.5]);
    
    % Add each timestep as image in GIF
    drawnow
    frame = getframe(1);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    if currentTime - domaintransect.startTime + 1 == 1
        imwrite(imind,cm,filenametransect,'gif', 'DelayTime',0.1, 'Loopcount',inf);
    else
        imwrite(imind,cm,filenametransect,'gif','WriteMode','append', 'DelayTime',0.1);
    end
    
    
    clf;
    
end

% Path over time
y = latListTransect(latListTransect ~= 0);
x = longListTransect(longListTransect ~= 0);
z = zeros(size(x));
col = 1:length(x);  % This is the color, vary with x in this case.
surface([x;x],[y;y],[z;z],[col;col],...
    'facecol','no',...
    'edgecol','interp',...
    'linew',2);
axis([-76.1,-75.1,33.5,34.5]);
title('Transect Path Over Time');
saveas(gcf, 'transectpath.fig');

% Convert total coverage to percentage and plot
figure;
plot(totalCoverageTransect./numel(domaintransect.coverageMap));
saveas(gcf, 'transectcoverage.fig');

