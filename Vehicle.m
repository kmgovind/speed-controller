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

        end

        function obj = moveBoat(obj, environment, goalLat, goalLong, currentTime)
            %obj is the current boat we are moving

            % Calculate new boat speed every 30 minutes (for MPC
            % implementation)
%             if mod(currentTime, 30) == 0
%                 obj.motorSpeed = obj.velocityCalc(environment, currentTime);
%             end
              obj.motorSpeed = obj.velocityCalc(environment, currentTime);

            if obj.charge == 0
                obj.motorSpeed = 0;
            end

            % Compute heading
            goalHeading = obj.headingCalc(goalLat, goalLong);

            % Pull flow components
            [flow_u, flow_v] = environment.flowComponents(obj.latitude, obj.longitude, currentTime); % Pull flow components from environmental data
            if isnan(flow_u)
                flow_u = 0;
            end
            if isnan(flow_v)
                flow_v = 0;
            end
            [flowspeed, flowheading] = obj.flowHeading(flow_u, flow_v);


            %             % Compute velocity for constant true speed case
            %             goal_u = convvel(4.5, 'kts', 'm/s') * sind(goalHeading);
            %             goal_v = convvel(4.5, 'kts', 'm/s') * cosd(goalHeading);
            %
            %             obj.motorSpeed_u = goal_u - flow_u;
            %             obj.motorSpeed_v = goal_v - flow_v;
            %             obj.motorSpeed = sqrt(obj.motorSpeed_u^2 + obj.motorSpeed_v^2);


            % Identify velocity components and add to flow components\
            if obj.motorSpeed ~= 0
                obj.heading = goalHeading + asind(-(flowspeed/obj.motorSpeed) * sind(mod(flowheading - goalHeading, 360)));
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
            
            if obj.speed_u == 0 && obj.speed_v == 0
                keyboard;
            end



            % Calculate resulting position only if battery can handle it
            % u moves longitude; v moves latitude
            obj.longitude = obj.longitude + km2deg((convvel(obj.speed_u, 'm/s', 'km/h') * hours(environment.timeStep)));
            obj.latitude = obj.latitude + km2deg((convvel(obj.speed_v, 'm/s', 'km/h') * hours(environment.timeStep)));

            % Update battery state
            obj.charge = useCharge(obj, environment, currentTime);

        end
        function updatedCharge = useCharge(obj, environment, currentTime)
            % Speed/Power Lookup Table
            speeds = [0, 2.4, 3.1, 3.83, 4.43, 4.9]; % boat speeds in kts
            speeds = convvel(speeds, 'kts', 'm/s');
            powerDraw = [0, 62, 115, 235, 404, 587]; % corresponding wattage

            motorDraw = interp1(speeds, powerDraw, obj.motorSpeed);
            %             irradiance = 0;
            irradiance = environment.getIrradiance(currentTime); % average irradiance in w/m^2
            irradiance = irradiance * obj.panelArea * obj.panelEfficiency; % getting wattage for solar panel generation


            updatedCharge = obj.charge + (irradiance - motorDraw - obj.backgroundDraw) * (hours(environment.timeStep));
            if updatedCharge <= 0
                updatedCharge = 0;
            elseif updatedCharge >= obj.batteryCapacity
                updatedCharge = obj.batteryCapacity;
            end
        end

        function heading = headingCalc(obj, goalLat, goalLong)
            latDif = goalLat - obj.latitude;
            longDif = goalLong - obj.longitude;
            heading = mod(180 * atan2(longDif, latDif) / pi, 360);
        end

        function [speed, heading] = flowHeading(obj, flowu, flowv)
            speed = sqrt(flowu.^2 + flowv.^2);
            heading = mod(180 .* atan2(flowu, flowv)./pi, 360);
        end

        function speed = velocityCalc(obj, environment, currentTime)
            %             speed = ((obj.panelEfficiency*obj.panelArea*irradiance) - obj.backgroundDraw);
            %             rho_sw = 1023.6; % density of salt water
            %             numerator = (obj.panelEfficiency * obj.panelArea * irradiance - obj.backgroundDraw)*obj.motorEfficiency;
            %             denominator = (1/2)*(rho_sw) * obj.boatArea * obj.Cd;
            %             speed = power(numerator/denominator,1/3);
            %             speed = abs(speed);

            % Strat: aim to deplete battery (using only motor power)
            % exactly one day from current time

            % Speed/Power Lookup Table
            speeds = [0, 2.4, 3.1, 3.83, 4.43, 4.9]; % boat speeds in kts
            speeds = convvel(speeds, 'kts', 'm/s');
            powerDraw = [0, 62, 115, 235, 404, 587]; % corresponding wattage

            SoC = obj.charge; % current charge in wH
            irradiance = environment.getIrradiance(currentTime); % average irradiance in w/m^2
            irradiance = irradiance * obj.panelArea * obj.panelEfficiency; % getting wattage for solar panel generation


            %             stateOfCharge = stateOfCharge/24; % current draw to hit goal
            %
            %             speed = interp1(powerDraw, speeds, stateOfCharge);

            % 0 change in SoC
            %             if irradiance >= 587
            %                 speed = convvel(4.5, 'kts', 'm/s');
            %             else
            %                 speed = interp1(powerDraw, speeds, irradiance);
            %             end


            % Constant 4.5 kts
                        if stateOfCharge > 0
                            speed = convvel(4.5, 'kts', 'm/s');
                        else
                            speed = 0;
                        end
%                         speed = convvel(4.5, 'kts', 'm/s');
            



            % MPC
%             dt = 30; % MPC Timestep (min) - max is 60
%             pred_hor = 48; % Horizon is 48 timesteps (24 hours) - min is 12
%             x0 = ones(1, pred_hor) * convvel(2.5, 'kts', 'm/s'); % Initial speed guess
%             A = [];
%             b = [];
%             Aeq = [];
%             beq = [];
%             lb = zeros(1, pred_hor);
%             ub = ones(1, pred_hor) * convvel(4.5, 'kts', 'm/s');
%             opts = optimoptions('fmincon','Display','none');
% 
% %             tic
%             xOpt = fmincon(@(x) -J_ASV(x, dt, obj, environment, currentTime), ...
%                 x0, A, b, Aeq, beq, lb, ub, [], opts);
% %             toc
% 
%             speed = xOpt(1);
% %             if speed == 0
% %                 keyboard
% %             end

            % Cap speed at 4.5kts
            if speed > convvel(4.5, 'kts', 'm/s')
                speed = convvel(4.5, 'kts', 'm/s');
            end


        end

    end
end

function out = J_ASV(x, dt, obj, environment, time)
% Constants

speeds = [0, 2.4, 3.1, 3.83, 4.43, 4.9]; % boat speeds in kts
speeds = convvel(speeds, 'kts', 'm/s');
powerDraw = [0, 62, 115, 235, 404, 587]; % corresponding wattage


dist = sum(dt*x*60); % Calculate possible travel distance (m) in Dt
% get flow speed from environment, add to motor speed, calculate distance
% using true velocity

% keep track of next waypoint, once you cover enough distance to cross the
% waypoint, update your waypoint to the next one

% need to convert speed from m/s to m/min
%     SoC_end = SoC - sum(0.5*boatArea*Cd*rho*x.^3) + energy_gain; % assume one hour
energy_gain = environment.getIrradiance(time);

% mpc v2 trial one
% Compute distance inside of this for loop
SoC_end = zeros(numel(x) + 1, 1);
SoC_end(1) = obj.charge;
for i = 1:numel(x)
    if i == 1
        SoC_end(i) = obj.charge + (environment.getIrradiance(time) - interp1(speeds, powerDraw, x(i))) * hours(minutes(dt));
    else
        SoC_end(i) = SoC_end(i-1) + (environment.getIrradiance(time + dt*(i-1)) - interp1(speeds, powerDraw, x(i))) * hours(minutes(dt));
    end
end


% SoC_end = SoC + (energy_gain - interp1(speeds, powerDraw, x)) * hours(minutes(dt))

% estimate additional distance value of energy remaining after first Dt
% assuming that we travel at the maximum possible boat speed and use up all
% the energy without charging
max_speed = convvel(4.5, 'kts', 'm/s'); % tune this value later
max_draw = interp1(speeds, powerDraw, max_speed); % power draw of max speed
max_speed = max_speed * 60 * 60; % speed in m/hr also distance traveled in one hr
distPerSoC = SoC_end(end)/max_draw; % number of hrs of travel at max speed based on the final amount of battery after n steps
distPerSoC = distPerSoC * max_speed;


J_inf = distPerSoC; % J_stored is infinite horizon prediction
% J_stored = 0;
% out = 0.95 * dist + 0.05 * J_stored;
out = dist + J_inf;
%     keyboard
end