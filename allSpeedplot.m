clear;clc;close all;

load("allSpeeds.mat");



% Plot speeds vs time
figure(1);
resultsTable.totalTime = hours(minutes(resultsTable.totalTime));
scatter3(resultsTable,'legOneSpeed','legTwoSpeed','totalTime','filled', ...
    'ColorVariable','usedCharge');
title('Total Traversal Time');
xlabel('Leg One Speed (kts)');
ylabel('Leg Two Speed (kts)');
zlabel('Travel Time (hrs)');
c = colorbar;
c.Label.String = 'Energy Used (Wh)';

% Plot speeds vs energy usage
figure(2);
scatter3(resultsTable,'legOneSpeed','legTwoSpeed','usedCharge','filled', ...
    'ColorVariable','totalTime');
title('Energy Used');
xlabel('Leg One Speed (kts)');
ylabel('Leg Two Speed (kts)');
zlabel('Energy Used (Wh)');
c = colorbar;
c.Label.String = 'Time taken (hrs)';

% Create combined metric of time and energy usage
resultsTable.totalTime = minutes(hours(resultsTable.totalTime));
resultsTable.comTimeEnergy = resultsTable.totalTime + resultsTable.usedCharge;

figure(3);
scatter3(resultsTable,'legOneSpeed','legTwoSpeed','comTimeEnergy','filled', ...
    'ColorVariable','comTimeEnergy');
title('Combined Time and Energy');
xlabel('Leg One Speed (kts)');
ylabel('Leg Two Speed (kts)');
zlabel('Combined Time/Energy');

[~, ix] = min(resultsTable.comTimeEnergy);
fprintf("Minimized values:\n");
fprintf("Leg One Speed:%d\tLeg Two Speed:%d\t\n", resultsTable.legOneSpeed(ix), resultsTable.legTwoSpeed(ix));
fprintf("Total Time:%d\tUsed Charge:%d\n", resultsTable.totalTime(ix), resultsTable.usedCharge(ix));
