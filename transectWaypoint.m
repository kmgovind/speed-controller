function [goalLat, goalLong] = transectWaypoint(legCount, boat, environment)



switch(legCount)
    case 0 %right
        goalLat = boat.latitude;
        goalLong = environment.longitudeRange(end) - km2deg(5);

    case 1 %vertical
        goalLat = boat.latitude + km2deg(10);
        if goalLat > environment.latitudeRange(end) - km2deg(5)
            goalLat = environment.latitudeRange(end) - km2deg(5);
        end
        goalLong = boat.longitude;
        
    case 2 %left      
        goalLat = boat.latitude;
        goalLong = environment.longitudeRange(1) + km2deg(5);

    case 3 %vertical
        goalLat = boat.latitude + km2deg(10);
        if goalLat >= environment.latitudeRange(end) - km2deg(5)
            goalLat = environment.latitudeRange(1) + km2deg(5);
            goalLong = environment.longitudeRange(1) + km2deg(5);
        else
            goalLong = boat.longitude;
        end
end

end
