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
        
        function [obj, chargeUsage] = moveBoat(obj, environment, goalLat, goalLong, currentTime, chargeUsage, legCounter)
            %obj is the current boat we are moving
            
            % Collect Boat assigned speed and heading towards goal
            obj.motorSpeed = obj.velocityCalc(legCounter);
            goalHeading = obj.headingCalc(goalLat, goalLong);
            
            % Don't let boat run motor if it will use up too much battery
%             if useCharge(obj, environment) <= 0
%                 obj.motorSpeed = 0;
%             end
            
            % Pull flow components
            [flow_u, flow_v] = environment.flowComponents(); % Pull flow components from environmental data
            if isnan(flow_u)
                flow_u = 0;
            end
            if isnan(flow_v)
                flow_v = 0;
            end
            [flowspeed, flowheading] = obj.flowHeading(flow_u, flow_v);
            
            % Compute heading that compensates for flow and points boat
            % towards goal
            obj.heading = goalHeading + asind(-(flowspeed/obj.motorSpeed) * sind(mod(flowheading - goalHeading, 360)));
            % TODO: Optimize lower-level strategy
            
            % Identify velocity components and add to flow components
            obj.motorSpeed_u = obj.motorSpeed * sind(obj.heading);
            obj.motorSpeed_v = obj.motorSpeed * cosd(obj.heading);
            
            obj.speed_u = obj.motorSpeed_u + flow_u;
            obj.speed_v = obj.motorSpeed_v + flow_v;
            
            % Calculate resulting position only if battery can handle it
            % u moves longitude; v moves latitude
            obj.longitude = real(obj.longitude + nm2deg((obj.speed_u * hours(environment.timeStep))));
            obj.latitude = real(obj.latitude + nm2deg((obj.speed_v * hours(environment.timeStep))));
            
%             % Update battery state
            chargeUsage = chargeUsage + useCharge(obj, environment, currentTime);
            
        end
        function updatedCharge = useCharge(obj, environment, currentTime)
            % Speed/Power Lookup Table
            speeds = [0, 2.4, 3.1, 3.83, 4.43, 4.9]; % boat speeds in kts
            speeds = convvel(speeds, 'kts', 'm/s');
            powerDraw = [0, 62, 115, 235, 404, 587]; % corresponding wattage
            
            motorDraw = interp1(speeds, powerDraw, obj.motorSpeed);
            %             irradiance = environment.getIrradiance(obj.latitude, obj.longitude);
%             irradiance = environment.getIrradiance(currentTime); % average irradiance in w/m^2
%             irradiance = irradiance * obj.panelArea * obj.panelEfficiency; % getting wattage for solar panel generation
            
%             updatedCharge = obj.charge + (irradiance - motorDraw - obj.backgroundDraw) * (hours(environment.timeStep));
%             if updatedCharge <= 0
%                 updatedCharge = 0;
%             elseif updatedCharge >= obj.batteryCapacity
%                 updatedCharge = obj.batteryCapacity;
%             end
            updatedCharge = motorDraw;
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
        
        function speed = velocityCalc(obj, legCounter)           
%             % Speed/Power Lookup Table
%             speeds = [0, 2.4, 3.1, 3.83, 4.43, 4.9]; % boat speeds in kts
%             speeds = convvel(speeds, 'kts', 'm/s');
%             powerDraw = [0, 62, 115, 235, 404, 587]; % corresponding wattage
%             
%             stateOfCharge = obj.charge; % current charge in wH
%             stateOfCharge = stateOfCharge/24; % current draw to hit goal
%             
%             speed = interp1(powerDraw, speeds, stateOfCharge);
%             
%             
%             if speed > convvel(4.5, 'kts', 'm/s')
%                 speed = convvel(4.5, 'kts', 'm/s');
%             end
            if legCounter == 1
                speed = convvel(2.5, 'kts', 'm/s');  
            elseif legCounter == 2
                speed = convvel(4.5, 'kts', 'm/s');
            end
        end
        
    end
end