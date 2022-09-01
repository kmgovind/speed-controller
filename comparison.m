% Driver Code
clear;clc;close all;

tic

load("legTwoLookup_1_5kts.mat");
lookupTbl = squeeze(struct2cell(results));
lookupTbl = cell2mat(lookupTbl);
clear results;

results(1).speedOne = nan;
results(1).speedTwo = nan;
results(1).time = nan;
results(1).legOneTime = nan;
results(1).legTwoTime = nan;
results(1).charge = nan;
results(1).legOneCharge = nan;
results(1).legTwoCharge = nan;
legOneCharges = 0.1:0.1:4.5;
counter = 0;

speeds = [0, 1, 2, 2.4, 3.1, 3.83, 4.43, 4.9]; % boat speeds in kts
speeds = convvel(speeds, 'kts', 'm/s');
powerDraw = [0, 18, 45, 62, 115, 235, 404, 587]; % corresponding wattage
chargeDiffThresh = 100;

for legOneSpeed = 2:0.1:4.5
    % Pre-defined vars
    global currentTime;
    wpThresh = 0.05;
    endRun = false; % trigger to end run
    counter = counter + 1;
    legCounter = 1;

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
    legTwoDist = deg2km(distance('gc', goal.lat(1), goal.long(1), goal.lat(2), goal.long(2)));
    legTwoDist = 1000 * legTwoDist; % legTwoDistance km to m
    legTwoSpeed = nan;


    % simulate leg 1 at various speeds
    %     legCounter = 1;
    %     while(~endRun)
    %         runVehicle = runVehicle.moveBoat(runDomain, goal.lat(legCounter), goal.long(legCounter), currentTime, legOneSpeed);
    %         [endRun, currentTime, legCounter] = endCheck(currentTime, runVehicle, runDomain, goal, wpThresh, legCounter);
    %     end
    %     results(end+1).speedOne = legOneSpeed;
    %     results(end).legOneTime = currentTime;
    %     results(end).legOneCharge = runVehicle.charge;
    %
    %     % determine charge usage for leg 2 at various speeds
    %     legCounter = 2;
    %     endRun = false;
    %     currentTime = runDomain.startTime;
    %     runVehicle.latitude = goal.lat(1);
    %     runVehicle.longitude = goal.long(1);
    %     while(~endRun)
    %         legTwoSpeed = legOneSpeed;
    %         if legTwoSpeed > 1.6
    %             runVehicle = runVehicle.moveBoat(runDomain, goal.lat(legCounter), goal.long(legCounter), currentTime, legTwoSpeed);
    %             [endRun, currentTime, legCounter] = endCheck(currentTime, runVehicle, runDomain, goal, wpThresh, legCounter);
    %         else
    %             endRun = true;
    %             runVehicle.charge = nan;
    %         end
    %     end
    %     results(end+1).speedTwo = legTwoSpeed;
    %     results(end).legTwoTime = currentTime;
    %     results(end).legTwoCharge = runVehicle.charge;


    while(~endRun)
        if legCounter == 1
            runVehicle = runVehicle.moveBoat(runDomain, goal.lat(legCounter), goal.long(legCounter), currentTime, legOneSpeed);
            [endRun, currentTime, legCounter] = endCheck(currentTime, runVehicle, runDomain, goal, wpThresh, legCounter);
            if legCounter == 2
                legOneTime = currentTime;
                legOneCharge = runVehicle.charge;

                availableCharge = runVehicle.batteryCapacity - runVehicle.charge;
                testSpeed = 0.1:0.1:4.5;
                testSpeed = convvel(testSpeed, 'kts', 'm/s');
                [flow_u, flow_v] = runDomain.flowComponents();

                % replace with try-catch
%                 denom = real(sqrt((testSpeed.^2) - (flow_v.^2)));
                denom = flow_u + testSpeed;
                legTwoTime = legTwoDist./denom;
                legTwoTime = hours(seconds(legTwoTime));
                estChargeUse = interp1(speeds, powerDraw, testSpeed, 'pchip') .* legTwoTime;
                indices = abs(estChargeUse - availableCharge) < chargeDiffThresh;
                legTwoSpeed = testSpeed(find(indices==1));

                
                
%                 keyboard

                if isempty(legTwoSpeed)
                    legTwoSpeed = nan;
                else
                    legTwoSpeed = legTwoSpeed(end);
                end
                legOneSpeed
                legTwoSpeed
%                 if availableCharge > 1000
%                     [ d, ix ] = min( abs( lookupTbl(2,:) - availableCharge ) );
%                     legTwoSpeed = lookupTbl(1,ix);
%                 else
%                     legTwoSpeed = nan;
%                 end
            end
        else
            if isnan(legTwoSpeed)
                endRun = true;
            else
                runVehicle = runVehicle.moveBoat(runDomain, goal.lat(legCounter), goal.long(legCounter), currentTime, legTwoSpeed);
                [endRun, currentTime, legCounter] = endCheck(currentTime, runVehicle, runDomain, goal, wpThresh, legCounter);
            end
        end
        location(1,currentTime) = runVehicle.latitude;
        location(2, currentTime) = runVehicle.longitude;
    end
    if ~isnan(legTwoSpeed)
        results(end+1).speedOne = legOneSpeed;
        results(end).speedTwo = legTwoSpeed;
        results(end).time = hours(minutes(currentTime));
        results(end).legOneTime = legOneTime;
        results(end).charge = runVehicle.charge;
        results(end).legOneCharge = legOneCharge;
    end
end

toc


% Plotting
results = squeeze(struct2cell(results));
legOneSpeeds = cell2mat(results(1,:));
times = cell2mat(results(3,:));

figure(1);
scatter(legOneSpeeds, times);
title('Time (hrs) vs Leg One Speed (kts)');
xlabel('Leg One Speed (kts)');
ylabel('Time (hrs)');
