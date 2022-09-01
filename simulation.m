% Driver Code
clear;clc;close all;

generateSummary = true;

% Pre-defined vars
global currentTime;
wpThresh = 0.05;
endRun = false; % trigger to end run
legCounter = 1; % counter for upstream vs cross-stream leg

% Initialize run
runDomain = Environment();
currentTime = runDomain.startTime;
start.long = -75.9;
start.lat = 33.7;
runVehicle = Vehicle(start.lat, start.long);

% Set goal
goal.long(1) = -75.9;
goal.lat(1) = 34.3;
goal.long(2) = -75.3;
goal.lat(2) = 34.3;

while(~endRun)
    % TODO: Plotting
    runVehicle = runVehicle.moveBoat(runDomain, goal.lat(legCounter), goal.long(legCounter), currentTime, legCounter);
    if legCounter == 1
        [endRun, currentTime, legCounter] = endCheck(currentTime, runVehicle, runDomain, goal, wpThresh, legCounter);
        if legCounter == 2
            legOneTime = currentTime;
            legOneCharge = runVehicle.charge;
        end
    else
        [endRun, currentTime, legCounter] = endCheck(currentTime, runVehicle, runDomain, goal, wpThresh, legCounter);
    end
    location(1,currentTime) = runVehicle.latitude;
    location(2, currentTime) = runVehicle.longitude;
end

% % plot path
% figure(1);
% hold on;
% plot(location(2,:), location(1,:));
% plot(start.long, start.lat, 'r*');
% plot(goal.long(2), goal.lat(2), 'g*');
% axis([-76.1, -75.1, 33.5, 34.5]);

%% Generate summary statistics
if generateSummary
    date = clock;
    path = 'C:\Users\kavin\Documents\Research\persistent-asv\speed-controller\summaryData';
    filename = sprintf('%d_%d_%d_%d_%d_summary.txt', date(1), date(2), date(3), date(4), date(5));
    fileID = fopen(fullfile(path, filename), 'w');

    % Run Info
    fprintf(fileID, 'Velocity Controller Test\n');
    fprintf(fileID, 'Year: %d\tMonth: %d\tDay: %d\n', date(1), date(2), date(3));
    fprintf(fileID, 'Hour: %d\tMinute: %d\n', date(4), date(5));

    % Update with actual strategy
    fprintf(fileID, '\n--------------------------------\n');
    fprintf(fileID, 'Strategy: Slow Leg One, Fast Leg Two\n');
    fprintf(fileID, 'Leg One Boat Speed: %0.2f m/s\n', convvel(2.5, 'kts', 'm/s'));
    fprintf(fileID, 'Leg Two Boat Speed: %0.2f m/s\n', convvel(4.5, 'kts', 'm/s'));
    fprintf(fileID, '--------------------------------\n');

    % Parameters/Results
    hour = hours(minutes(currentTime));
    minute = (hour - floor(hour))*60;
    fprintf(fileID, 'Total Duration:\n\tHours: %d\tMinutes: %d\n', floor(hour), round(minute));
    fprintf(fileID, 'Total Charge Usage: %0.2f Wh\n', runVehicle.charge);
    [flow_u, flow_v] = runDomain.flowComponents();
    fprintf(fileID, 'Flow U: %0.2f m/s\tFlow V: %0.2f m/s\n', flow_u, flow_v);

    fprintf(fileID, '\n--------------------------------\n');
    hour = hours(minutes(legOneTime));
    minute = (hour - floor(hour)) * 60;
    fprintf(fileID, 'Leg 1 Duration:\n\tHours: %d\tMinutes: %d\n', floor(hour), round(minute));
    fprintf(fileID, 'Leg One Charge Usage: %0.2f Wh\n', legOneCharge);
    legTwoTime = currentTime - legOneTime;
    hour = hours(minutes(legTwoTime));
    minute = (hour - floor(hour)) * 60;
    fprintf(fileID, 'Leg 2 Duration:\n\tHours: %d\tMinutes: %d\n', floor(hour), round(minute));
    fprintf(fileID, 'Leg Two Charge Usage: %0.2f Wh\n', runVehicle.charge - legOneCharge);
    fclose(fileID);
end
