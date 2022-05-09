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
        function obj = Environment(desiredSpeed, desiredDays)
            % Calculate total nm width given speed and days
            numHours = hours(days(desiredDays)); % number of hours in desiredDays
            totalnm = desiredSpeed * numHours;
            totaldeg = nm2deg(totalnm); % Convert nm to degrees
            obj.endTime = minutes(days(desiredDays));
            obj.longitudeRange = 0:0.01:totaldeg;
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

        function sunlight = avgIrradiance(obj)
            global currentTime;
            %             flowTime = days(minutes(currentTime));
            %             sunlight = interp1(obj.enviroData.sun_t, obj.avgSun, flowTime);

            %             % Continuous interpolation of irradiance data based on time
            %             sunlight = interp1(obj.enviroData.sun_t, obj.avgSun, currentTime);

            % Find index in obj.enviroData.sun_t that is nearest to
            % currentTime and reference value in obj.avgSun
            [~, ix] = min(abs(obj.enviroData.sun_t - currentTime));
            sunlight = obj.avgSun(ix);
        end

        function [obj, coverageSum] = updateCoverage(obj, boatLat, boatLong)

            % Initialize coverageSum
            coverageSum = 0;

            % Update coverage at each gridpoint
            dataCell = struct2cell(obj.coverageMap);
            dataMatrix = cell2mat(dataCell);
            lats = squeeze(dataMatrix(1,:,:));
            longs = squeeze(dataMatrix(2,:,:));
            coverages = squeeze(dataMatrix(3,:,:));


            % Coverage Loss
            % Currently 0.01% of current coverage
            lossConstant = 0.01/100; % 0.01 percent
            coverages = (1-lossConstant)*coverages;


            %             offset = deg2nm(distance('gc',gridLat, gridLong, boatLat, boatLong));

            nmPerLong = 60*cosd(34.5);
            nmPerLat = 60;
            offset = sqrt((nmPerLat*lats-nmPerLat*boatLat).^2 + (nmPerLong*longs - nmPerLong*boatLong).^2); %dist in nm


            % Update Coverages
            %             coverages = max(coverages,1./(offset.^2+1));

            %             exp(-dist^2/2*10^2)
            lengthScale = km2nm(10); %10 km length scale converted to nm
            %             lengthScale = 10;
            coverages = max(coverages, exp( (-offset.^2)./(2*lengthScale^2)));


            % Update Coverage Sum
            % Compute coverage sum
            coverageSum = sum(coverages,'all');

            T = num2cell(coverages);
            [obj.coverageMap(:,:).coverage] = T{:};

        end
    end
end