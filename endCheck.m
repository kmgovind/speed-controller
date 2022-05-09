function [endRun, currentTime] = endCheck(currentTime, runVehicle, runDomain, goal, wpThresh)
%endCheck Check whether the simulation end trigger should be tripped
%   endRun - simulation end trigger (initially false to let sim run)
%   currentTime - currentTime of the simulation (end if past endTime)
%   runVehicle - current state of the vehicle
%   runDomain - current state of the domain
%   goal - location boat is attempting to reach
%   wpThresh - how far away from the goal boat needs to be to be considred
%   as having reached the actual goal
    endRun = false;

    lat = runVehicle.latitude;
    long = runVehicle.longitude;

    latRange = runDomain.latitudeRange;
    longRange = runDomain.longitudeRange;

    % End simulation if boat is out of bounds
    if lat < latRange(1) | lat > latRange(end)
        endRun = true;
    end
    if long < longRange(1) | long > longRange(end)
        endRun = true;
    end

    % End simulation if boat has reached goal
    if distanceCalc(lat, long, goal) <= wpThresh
        endRun = true;
    end
    
    % Iterate time step
    if currentTime >= runDomain.endTime
        endRun = true; % end run if time is up
    else
        currentTime = currentTime + 1;
    end
end

function dist = distanceCalc(currentLat, currentLong, goal)
    dist = deg2nm(distance('gc', goal.lat, goal.long, currentLat, currentLong));
end