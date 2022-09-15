classdef Environment
    % Properties of environment

    properties
        enviroData; % flow and sunlight data
    end

    properties (Constant)
        % Time
        timeStep = minutes(1);
        startTime = minutes(days(1)); % Start at 1 Day
        endTime = minutes(days(60)); % End at 60 Day
        %         endTime = minutes(days(3));
        %         latitudeRange = 32:0.01:37;
        %         longitudeRange = -79:0.01:-74;
        %         latitudeRange = 32.5:0.01:33.5;
        %         longitudeRange = -78:0.01:-77;

        latitudeRange = 33.5:0.01:34.5;
        longitudeRange = -76.1:0.01:-75.1;
    end

    %% Methods
    methods (Static)
        function obj = Environment()
            obj.enviroData = load('2MonthData.mat');
            obj.enviroData.sun_t = 0:8*60:248*8*60;
            obj.enviroData.sun_ssr = mean(mean(obj.enviroData.sun_ssr));
            obj.enviroData.sun_ssr = squeeze(obj.enviroData.sun_ssr);
        end
    end

    methods
        function [flow_u, flow_v] = flowComponents(obj, latitude, longitude)

            % Interpolate data at required time
            global currentTime
            flowTime = days(minutes(currentTime));
            flow_u = interp3(obj.enviroData.flowlat, obj.enviroData.flowlon, obj.enviroData.flowt, obj.enviroData.u, latitude, longitude, flowTime);
            flow_v = interp3(obj.enviroData.flowlat, obj.enviroData.flowlon, obj.enviroData.flowt, obj.enviroData.v, latitude, longitude, flowTime);

            %             % Convert from m/s to kts
            %             flow_u = convvel(flow_u, 'm/s', 'kts');
            %             flow_v = convvel(flow_v, 'm/s', 'kts');
        end

        function sunlight = getIrradiance(obj)
            % find solar irradiance at given location at current time
            % Interpolate data at required time
            global currentTime
%             flowTime = days(minutes(currentTime));
%             sunlight = interp3(sort(double(obj.enviroData.sun_long)), sort(double(obj.enviroData.sun_lat)), sort(unique(double(obj.enviroData.sun_t))), double(obj.enviroData.sun_ssr), latitude, longitude, flowTime);
            sunlight = interp1(obj.enviroData.sun_t, obj.enviroData.sun_ssr, currentTime);
            if isnan(sunlight)
                sunlight = 0;
            end
        end

    end
end