% Driver Code
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
start.long = runDomain.longitudeRange(1);
start.lat = mean(runDomain.latitudeRange);
runVehicle = Vehicle(start.lat, start.long);

% Set goal 
goal.long = runDomain.longitudeRange(end);
goal.lat = start.lat; % travel perfectly horizontal

while(~endRun)
    

    [endRun, currentTime] = endCheck(currentTime, runVehicle, runDomain, goal, wpThresh);
end

