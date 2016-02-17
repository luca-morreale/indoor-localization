function [ beacon_indexes ] = poistion_selector( position, beacons_position, radius, qt)
% This function computes the distances bewtween each beacons and the 
% given position, and then extract the index of first _qt_ nearer beacons 

[sensor_size, N] = size(beacons_position);
distances = zeros(sensor_size, 1);

% computing distances
for k=1:sensor_size
    distances(k) = sqrt((position(1)-beacons_position(k,1)).^2 + ...
                        (position(2)-beacons_position(k,2)).^2);
    if distances(k) > radius
        distances(k) = 0;
    end
end

% removing zeros
valide_distances = distances(find(distances > 0));

[B, indexes] = sort(valide_distances);

beacon_indexes = indexes(1:min(qt, size(valide_distances,1)));

end

