classdef Environment
    % Properties of environment
    
    properties
        endTime;
        longitudeRange;
        irradiance;
        sunTime;
    end
    
    properties (Constant)
        % Time
        startTime = 0;
        timeStep = minutes(1);
        
        % Domain
        latitudeRange = 0:0.01:0.331;
    end
    
    %% Methods
    methods (Static)
        
        % Initialize Domain
        function obj = Environment(desiredSpeed, desiredDays)
            % Calculate total nm width given speed and days
            numHours = hours(days(desiredDays)); % number of hours in desiredDays
            totalnm = desiredSpeed * numHours;
            totaldeg = nm2deg(totalnm); % Convert nm to degrees
            obj.endTime = minutes(days(desiredDays));
            obj.longitudeRange = 0:0.01:totaldeg;

            enviroData = load('flowSunData3.mat');
            obj.sunTime = 0:8*60:248*8*60;
            avgSun = mean(enviroData.sun_ssr, [1,2]);
            obj.irradiance = squeeze(avgSun);
        end
        
        % Flow Dynamics
        function [flow_u, flow_v] = flowComponents()
            % Constant vertical flow at 2 m/s
            flow_u = 0;
            flow_v = 2;
        end
    end
    
    methods
        function sunlight = getIrradiance(obj, currentTime)
            % Find index in obj.enviroData.sunTime that is nearest to
            % currentTime and reference value in obj.avgSun
            [~, ix] = min(abs(obj.sunTime - currentTime));
            sunlight = obj.irradiance(ix);
        end
    end
end