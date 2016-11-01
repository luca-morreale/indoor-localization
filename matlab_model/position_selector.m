function [ beacon_indexes ] = position_selector( estimated_position, P_hat, basestation_positions, qt, measurement_weighted)
% This function computes the distances bewtween each beacons and the 
% given position, and then extract the index of first _qt_ nearer beacons 

sensor_size = length(basestation_positions);
distances = zeros(sensor_size, 1);

% computing distances
for k=1:sensor_size

    A = (estimated_position - [basestation_positions(k,:), 0, 0]);
    if not(measurement_weighted)
        distances(k) = A(1:2) * A(1:2)';

    else
        distances(k) = A * (P_hat \ A');

    end

end

% sorting distances
[~, indexes] = sort(distances);

% extracting number of valide distances
valide_size = size(find(distances > 0), 1) - 1;

% calculate the minimum number of beacon to consider
offset = min(valide_size, qt-1);
% calculate the first index of a valide beacon
first_index  = size(indexes,1)-valide_size;

% extract indexes
beacon_indexes = indexes(first_index : first_index + offset);

end

