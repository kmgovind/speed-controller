classdef Environment
    % Properties of environment
    
    properties
        endTime;
        longitudeRange;
    end
    
    properties (Constant)
        % Time
        startTime = 0;
        
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
        end
        
        % Flow Dynamics
        function [flow_u, flow_v] = flowComponents()
            % Constant vertical flow at 2 m/s
            flow_u = 0;
            flow_v = 2;
        end
    end
    
    methods
%         TODO: Add sunlight dynamics
    end
end