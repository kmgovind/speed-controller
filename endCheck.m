function [endRun, currentTime] = endCheck(currentTime, runVehicle, runDomain)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
    endRun = false;
    
    lat = runVehicle.latitude;
    long = runVehicle.longitude;

    latRange = runDomain.latitudeRange;
    longRange = runDomain.longitudeRange;

    if lat < latRange(1) | lat > latRange(end)
        endRun = true;
    end
    if long < longRange(1) | long > longRange(end)
        endRun = true;
    end


    % Iterate time step
    if currentTime >= runDomain.endTime
        endRun = true; % end run if time is up
    else
        currentTime = currentTime + 1;
    end
end