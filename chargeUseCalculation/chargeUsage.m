clear;clc; close all;

% Summary
% Goal is to create a table with flow heading as column headers and with
% flow velocity as row headers. The values in the table should show the
% total battery expenditure to complete a certain distance in those
% conditions assuming boat is outputting a constant motor speed and is
% following a path of heading 090.

travelDistance = 5; % 5 km
dist_m = travelDistance * power(10,3);
goalHeading = 090;

% flow setup
flowHeadings = 0:45:315;
flowSpeeds = 0:0.1:2; % flowSpeeds in m/s
flowSpeeds = flowSpeeds';

% boat setup
motorSpeed = convvel(4.5, 'kts', 'm/s'); % boats motor speed
speeds = [0, 2.4, 3.1, 3.83, 4.43, 4.9]; % boat speeds in kts
speeds = convvel(speeds, 'kts', 'm/s');
powerDraw = [0, 62, 115, 235, 404, 587]; % corresponding wattage
motorDraw = interp1(speeds, powerDraw, motorSpeed);

for heading = flowHeadings
    
    % flow components
    boatHeading = goalHeading + asind(-(flowSpeeds./motorSpeed) .* sind(mod(heading - goalHeading, 360)));
    motorSpeed_u = motorSpeed .* sind(boatHeading);
    flow_u = flowSpeeds .* sind(heading);
    total_u = motorSpeed_u + flow_u;
    
    travelTime = dist_m ./ total_u; % travel time in s
    travelTime = travelTime ./ (60 * 60); % travel time in hours

    chargeUse = motorDraw .* travelTime;
    assignin('base',['deg', num2str(heading)],chargeUse);
end

% data management
charges = table(flowSpeeds, deg0, deg45, deg90, deg135, deg180, deg225, deg270, deg315);
filename = sprintf('dist%dkm.xlsx', travelDistance);
writetable(charges, filename);