function [endRun, currentTime, legCounter] = endCheck(currentTime, runVehicle, runDomain, goal, wpThresh, legCounter)
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
    if distanceCalc(lat, long, goal, legCounter) <= wpThresh
        if legCounter == 2
            endRun = true;
        else
            legCounter = 2;
        end
    end
    
    currentTime = currentTime + 1;
end

function dist = distanceCalc(currentLat, currentLong, goal, legCounter)
    dist = deg2nm(distance('gc', goal.lat(legCounter), goal.long(legCounter), currentLat, currentLong));
end