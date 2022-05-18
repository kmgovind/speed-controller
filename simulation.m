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

% Tracking variables
batteryState = zeros(1,minutes(days(desiredDays)));
location = zeros(2,minutes(days(desiredDays)));

while(~endRun)
    % TODO: Plotting
    runVehicle = runVehicle.moveBoat(runDomain, goal.lat, goal.long, currentTime);
    [endRun, currentTime] = endCheck(currentTime, runVehicle, runDomain, goal, wpThresh);
    batteryState(currentTime) = runVehicle.charge;
    location(1,currentTime) = runVehicle.latitude;
    location(2, currentTime) = runVehicle.longitude;
end

figure(1);
subplot(2,1,1);
plot(location(2,:), location(1,:));
subplot(2,1,2);
plot(batteryState);
