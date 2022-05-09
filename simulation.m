clear;clc;close all;

% User vars
desiredSpeed = 5; % max speed of 5 kts
desiredDays = 5; % number of days to run sim for

% Pre-defined vars
wpThresh = 0.05;
endRun = false; % trigger to end run


% Initialize run
runDomain = Environment(desiredSpeed, desiredDays);
currentTime = runDomain.startTime;
startLong = runDomain.longitudeRange(1);
startLat = mean(runDomain.latitudeRange);
runVehicle = Vehicle(startLat, startLong);

while(~endRun)
    

    [endRun, currentTime] = endCheck(currentTime, runVehicle, runDomain);
end

