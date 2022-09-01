clear;clc;close all;

global currentTime;
wpThresh = 0.05;
counter = 0;

speeds = 1.7:0.1:4.5;
varTypes = {'double', 'double', 'double', 'double'};
varNames = {'legOneSpeed', 'legTwoSpeed', 'totalTime', 'usedCharge'};
resultsTable = table('Size', [numel(speeds)^2, 4], 'VariableTypes', varTypes, 'VariableNames', varNames);


for legOneSpeed = 1.7:0.1:4.5
    for legTwoSpeed = 1.7:0.1:4.5
        % Pre-defined vars
        endRun = false; % trigger to end run
        legCounter = 1; % counter for upstream vs cross-stream leg

        % Initialize run
        runDomain = Environment();
        currentTime = runDomain.startTime;
        start.long = -75.9;
        start.lat = 33.7;
        runVehicle = smartVehicle(start.lat, start.long);

        % Set goal
        goal.long(1) = -75.9;
        goal.lat(1) = 34.3;
        goal.long(2) = -75.3;
        goal.lat(2) = 34.3;

        while(~endRun)
            % TODO: Plotting
            if legCounter == 1
                runVehicle = runVehicle.moveBoat(runDomain, goal.lat(legCounter), goal.long(legCounter), currentTime, legOneSpeed);
                [endRun, currentTime, legCounter] = endCheck(currentTime, runVehicle, runDomain, goal, wpThresh, legCounter);
            else
                runVehicle = runVehicle.moveBoat(runDomain, goal.lat(legCounter), goal.long(legCounter), currentTime, legTwoSpeed);
                [endRun, currentTime, legCounter] = endCheck(currentTime, runVehicle, runDomain, goal, wpThresh, legCounter);
            end
        end
        
        % save results
        counter = counter + 1;
        resultsTable.legOneSpeed(counter) = legOneSpeed;
        resultsTable.legTwoSpeed(counter) = legTwoSpeed;
        resultsTable.totalTime(counter) = currentTime;
        resultsTable.usedCharge(counter) = runVehicle.charge;
    end
end

