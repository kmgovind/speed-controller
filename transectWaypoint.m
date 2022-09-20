function [goalLat, goalLong] = transectWaypoint(legCount, boat, environment)


startLat = environment.latitudeRange(1) + km2deg(5);
startLong = environment.longitudeRange(1) + km2deg(5);
endLat = environment.latitudeRange(end) - km2deg(5);
transectWidth = (endLat - startLat)/11; %width between transects in deg (running 10 transects)

switch(legCount)
    case 0 %right
        goalLat = boat.latitude;
        goalLong = environment.longitudeRange(end) - km2deg(5);

    case 1 %right hand side vertical
%         goalLat = boat.latitude + km2deg(8);
        goalLat = boat.latitude + transectWidth;
        if goalLat >= endLat
            goalLat = environment.latitudeRange(end) - km2deg(5);
        end
        goalLong = boat.longitude;
        
    case 2 %left      
        goalLat = boat.latitude;
        goalLong = environment.longitudeRange(1) + km2deg(12.5);
        if (goalLat + transectWidth) >= endLat
            goalLong = startLong;
        end

    case 3 % left hand side vertical
%         goalLat = boat.latitude + km2deg(8);
        goalLat = boat.latitude + transectWidth;
        if goalLat >= endLat
            goalLat = environment.latitudeRange(1) + km2deg(5);
            goalLong = environment.longitudeRange(1) + km2deg(5);
        else
            goalLong = boat.longitude;
        end
end

end

