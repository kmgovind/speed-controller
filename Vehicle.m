classdef Vehicle
    %% Properties of boat
    properties
        % Boat Position
        latitude; % Vehicle's current latitude
        longitude; % Vehicle's current longitude

        % Boat Net Speeds in kts
        speed; % Vehicle net speed (including flow)
        speed_u; % Vehicle net speed u component
        speed_v; % Vehicle net speed v component
        heading; % Vehicle's current heading relation to due north in degrees

        % Boat Motor Speeds in kts
        motorSpeed; % Vehicle's magnitude of speed from only motor
        motorSpeed_u; % Vehicle speed from motor in u direction
        motorSpeed_v; % Vehicle speed from motor in v direction
        motorEfficiency = 0.25; % efficiency of boat motor

        % Battery Details
        charge; % Vehicle's current battery charge
        powerFit; % power as a function of speed. Output is power
        speedFit; % speed as a function of power. Output is speed
        batteryCapacity = 6.5e3; % 6.5 kWh battery capacity
        backgroundDraw = 10; % 10W Hotel Load

        % Solar Panel Details
        panelArea = 4.16667; % Area of panel in m^2
        panelEfficiency = 0.18; % Efficiency of solar panel

        % Boat dimensions/constants
        Cd = 0.0030; % Coefficient of drag
        boatArea = 5.82; % Wetted area of boat in m^2


    end


    %% Methods
    methods
        function obj = Vehicle(lat, long)
            % Assign Lat-Long
            obj.latitude = lat;
            obj.longitude = long;

            % Instantiate Net Speeds
            obj.heading = 0;
            obj.speed = 0;
            obj.speed_u = 0;
            obj.speed_v = 0;

            % Instantiate Motor Speeds
            obj.motorSpeed = 0;
            obj.motorSpeed_u = 0;
            obj.motorSpeed_v = 0;

            % Instantiate Battery
            obj.charge = obj.batteryCapacity;

            speeds = [0, 2.4, 3.1, 3.83, 4.43, 4.9]; % boat speeds in kts
            speeds = convvel(speeds, 'kts', 'm/s');
            powerDraw = [0, 62, 115, 235, 404, 587]; % corresponding wattage
            obj.powerFit = polyfit(speeds, powerDraw, 3);
            obj.speedFit = polyfit(powerDraw, speeds, 3);
        end

        function obj = moveBoat(obj, environment, goalLat, goalLong, legCount, currentTime)
            %obj is the current boat we are moving

            % Compute heading
            goalHeading = obj.headingCalc(goalLat, goalLong, obj.latitude, obj.longitude);

            % Pull flow components
            [flow_u, flow_v] = environment.flowComponents(obj.latitude, obj.longitude, currentTime); % Pull flow components from environmental data
            if isnan(flow_u)
                flow_u = 0;
            end
            if isnan(flow_v)
                flow_v = 0;
            end
            [flowspeed, flowheading] = obj.flowHeading(flow_u, flow_v);

            % Compute new boat velocity

            % Calculate new boat speed every 30 minutes (for MPC
            % implementation)
            %             if mod(currentTime, 60) == 0
            %                 obj.motorSpeed = obj.velocityCalc(environment, legCount, goalLat, goalLong, currentTime);
            %             end
            obj.motorSpeed = obj.velocityCalc(environment, legCount, goalLat, goalLong,  currentTime);

            if obj.charge == 0
                obj.motorSpeed = 0;
            end

            % Identify velocity components and add to flow components\
            if obj.motorSpeed ~= 0
                obj.heading = goalHeading + real(asind(-(flowspeed/obj.motorSpeed) * sind(mod(flowheading - goalHeading, 360))));
                obj.motorSpeed_u = obj.motorSpeed * sind(obj.heading);
                obj.motorSpeed_v = obj.motorSpeed * cosd(obj.heading);

                obj.speed_u = obj.motorSpeed_u + flow_u;
                obj.speed_v = obj.motorSpeed_v + flow_v;
            else
                % if motor speed is 0, just get pushed by flow
                %                 keyboard;
                obj.speed_u = flow_u;
                obj.speed_v = flow_v;
            end

            % Calculate resulting position
            % u moves longitude; v moves latitude
            obj.longitude = obj.longitude + real(km2deg((convvel(obj.speed_u, 'm/s', 'km/h') * hours(environment.timeStep))));
            obj.latitude = obj.latitude + real(km2deg((convvel(obj.speed_v, 'm/s', 'km/h') * hours(environment.timeStep))));

            % Update battery state
            obj.charge = useCharge(obj, environment, currentTime);

        end
        function updatedCharge = useCharge(obj, environment, currentTime)
            motorDraw = polyval(obj.powerFit, obj.motorSpeed); % boat power draw at given speed
            irradiance = environment.getIrradiance(currentTime); % average irradiance in w/m^2
            irradiance = irradiance * obj.panelArea * obj.panelEfficiency; % getting wattage for solar panel generation

            updatedCharge = obj.charge + (irradiance - motorDraw - obj.backgroundDraw) * (hours(environment.timeStep));
            if updatedCharge <= 0
                updatedCharge = 0;
            elseif updatedCharge >= obj.batteryCapacity
                updatedCharge = obj.batteryCapacity;
            end
        end

        function heading = headingCalc(obj, goalLat, goalLong, latitude, longitude)
            latDif = goalLat - latitude;
            longDif = goalLong - longitude;
            heading = mod(180 * atan2(longDif, latDif) / pi, 360);
        end

        function [speed, heading] = flowHeading(obj, flowu, flowv)
            speed = sqrt(flowu.^2 + flowv.^2);
            heading = mod(180 .* atan2(flowu, flowv)./pi, 360);
        end

        function speed = velocityCalc(obj, environment, legCount, goalLat, goalLong,  currentTime)
            % Compute SoC and Irradiance
            SoC = obj.charge; % current charge in wH
            irradiance = environment.getIrradiance(currentTime); % average irradiance in w/m^2
            irradiance = irradiance * obj.panelArea * obj.panelEfficiency; % getting wattage for solar panel generation

            % Compute goal heading
            goalHeading = obj.headingCalc(goalLat, goalLong, obj.latitude, obj.longitude);

            % Pull flow components
            [flow_u, flow_v] = environment.flowComponents(obj.latitude, obj.longitude, currentTime); % Pull flow components from environmental data
            if isnan(flow_u)
                flow_u = 0;
            end
            if isnan(flow_v)
                flow_v = 0;
            end
            [flowspeed, flowheading] = obj.flowHeading(flow_u, flow_v);

            % Constant SoC Benchmark
            v_min = 0.5; % min true speed m/s
            % Compute velocity for constant true speed case
            goal_u = v_min * sind(goalHeading);
            goal_v = v_min * cosd(goalHeading);

            vbmin_u = goal_u - flow_u;
            vbmin_v = goal_v - flow_v;
            vbmin = sqrt(vbmin_u^2 + vbmin_v^2);
            
            v_pb = polyval(obj.speedFit, irradiance);

            speed = max(v_pb, vbmin);


            % 0 change in SoC
            %             if irradiance >= polyval(obj.powerFit, convvel(4.5, 'kts', 'm/s'))
            %                 speed = convvel(4.5, 'kts', 'm/s');
            %             else
            %                 speed = polyval(obj.speedFit, irradiance);
            %             end
            %             speed = max(speed, v_transect);


            % Constant 2.5 kts
            %             speed = convvel(2.5, 'kts', 'm/s');

            % Constant 4.5 kts
            %             speed = convvel(4.5, 'kts', 'm/s');
            %              speed = 2.1938;

            % Smart constant velocity
            %             v_target = 2.1938;
            %             soc_thresh = 500;
            %             if SoC - soc_thresh >= 0
            %                 v_soc = polyval(obj.speedFit, SoC - soc_thresh);
            %             else
            %                 v_soc = 0;
            %             end
            %             speed = median([v_target, v_soc, v_transect]);

            % MPC
            %             dt = 60; % MPC Timestep (min) - max is 60
            %             pred_hor = 12; % Horizon is 48 timesteps (24 hours) - min is 12
            %             x0 = ones(1, pred_hor) * convvel(2.5, 'kts', 'm/s'); % Initial speed guess
            %             A = [];
            %             b = [];
            %             Aeq = [];
            %             beq = [];
            %             lb = zeros(1, pred_hor);
            %             ub = ones(1, pred_hor) * convvel(4.5, 'kts', 'm/s');
            %             opts = optimoptions('fmincon','MaxIterations', 10, 'Display','none');
            %
            %             tic
            %             xOpt = fmincon(@(x) -J_ASV(x, dt, obj, environment, legCount, goalLat, goalLong, currentTime), ...
            %                 x0, A, b, Aeq, beq, lb, ub, [], opts);
            %             toc
            %
            %             speed = xOpt(1);
            %
            %             [flow_u, flow_v] = environment.flowComponents(obj.latitude, obj.longitude, currentTime);
            %             [flowspeed, ~] = obj.flowHeading(flow_u, flow_v);
            %
            %             speed = max([speed, flowspeed]);

            % Cap speed at 4.5kts
            if speed > convvel(4.5, 'kts', 'm/s')
                speed = convvel(4.5, 'kts', 'm/s');
            end


        end

    end
end

function out = J_ASV(x, dt, obj, environment, legCount, goalLat, goalLong, time)
% mpc v2 trial one
% Compute distance inside of this for loop
SoC_end = zeros(numel(x) + 1, 1);
SoC_end(1) = obj.charge;
dist = zeros(numel(x), 1);
latitude = obj.latitude;
longitude = obj.longitude;
% tic
for i = 1:numel(x)
    long_start = longitude;
    lat_start = latitude;
    [flow_u, flow_v] = environment.flowComponents(latitude, longitude, time + dt*(i-1));
    [flowspeed, flowheading] = obj.flowHeading(flow_u, flow_v);
    goalHeading = obj.headingCalc(goalLat, goalLong, latitude, longitude);
    heading = goalHeading + real(asind(-(flowspeed/x(i)) * sind(mod(flowheading - goalHeading, 360))));
    motorSpeed_u = x(i) * sind(heading);
    motorSpeed_v = x(i) * cosd(heading);

    speed_u = motorSpeed_u + flow_u;
    speed_v = motorSpeed_v + flow_v;



    % Calculate resulting position only if battery can handle it
    % u moves longitude; v moves latitude
    longitude = longitude + real(km2deg((convvel(speed_u, 'm/s', 'km/h') * hours(minutes(dt)))));
    latitude = latitude + real(km2deg((convvel(speed_v, 'm/s', 'km/h') * hours(minutes(dt)))));

    % Update waypoint if boat gets close enough
    if deg2nm(distance('gc',goalLat, goalLong, latitude, longitude)) <= 2.5
        legCount = mod(legCount + 1, 4);
        [goalLat, goalLong] = transectWaypoint(legCount, obj, environment);
    end

    % Store distance and charge at end of each timestep
    SoC_end(i+1) = min(6500, SoC_end(i) + (environment.getIrradiance(time + dt*(i-1)) - polyval(obj.powerFit, x(i))) * hours(minutes(dt)));
    if SoC_end(i+1) > 0
        dist(i) = deg2km(distance('gc', latitude, longitude, lat_start, long_start));
    else
        e_i = environment.getIrradiance(time + dt*(i-1));
        dist_pred = deg2km(distance('gc', latitude, longitude, lat_start, long_start));
        w_i = SoC_end(i) + e_i;
        w_icmd = SoC_end(i) - SoC_end(i+1) + e_i;
        dist(i) = (w_i/w_icmd) * dist_pred;
        SoC_end(i+1) = 0;
    end
end
% toc

% estimate additional distance value of energy remaining
t = 6; % 6 hours
p_avg = (SoC_end(1) - SoC_end(end))/12;
J_inf = t*(polyval(obj.speedFit, p_avg + (SoC_end(end)/t)) - polyval(obj.speedFit, p_avg));
out = sum(dist) + J_inf;
end