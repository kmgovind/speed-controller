classdef Environment
    % Properties of environment
    
    properties
        enviroData; % flow and sunlight data
        coverageMap; % initial coverage map
    end
    
    properties (Constant)
        % Time
        timeStep = minutes(1);
        startTime = minutes(days(0)); % Start at 1 Day
        latitudeRange = 33.5:0.01:34.5;
        longitudeRange = -76.1:0.01:-75.1;
    end
    
    %% Methods
    methods (Static)
        function obj = Environment()
            obj.enviroData = load('2MonthData.mat');
            obj.enviroData.sun_t = 0:8*60:248*8*60;
            obj.coverageMap = repmat(struct('latitude', 0, 'longitude', 0, 'coverage', 0.001), length(obj.latitudeRange), length(obj.longitudeRange));
%             obj.flow = repmat(struct('latitude', 0, 'longitude', 0, 'speed', 0, 'heading', 0), length(
            
            
            for lats = 1:length(obj.latitudeRange)
                for longs = 1: length(obj.longitudeRange)
                    obj.coverageMap(lats, longs).latitude = obj.latitudeRange(lats);
                    obj.coverageMap(lats,longs).longitude = obj.longitudeRange(longs);
                end
            end
        end
        function [flow_u, flow_v] = flowComponents()
            % Output constant flow in positive y-axis direction in m/s
            flow_u = convvel(1.5, 'kts', 'm/s');
            flow_v = convvel(1.5, 'kts', 'm/s');
        end
    end
    
    methods
        function sunlight = getIrradiance(obj, latitude, longitude)
            % find solar irradiance at given location at current time
            % Interpolate data at required time
            global currentTime
            flowTime = days(minutes(currentTime));
            sunlight = interp3(sort(double(obj.enviroData.sun_long)), sort(double(obj.enviroData.sun_lat)), sort(unique(double(obj.enviroData.sun_t))), double(obj.enviroData.sun_ssr), latitude, longitude, flowTime);
            if isnan(sunlight)
                sunlight = 0;
            end
        end
    end
end