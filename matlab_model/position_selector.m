function [ beacon_indexes ] = position_selector( estimated_position, P_hat, basestation_positions)
% This function computes the distances bewtween each beacons and the 
% given position, and then extract the index of nearer beacons 

sensor_size = length(basestation_positions);
distances = zeros(sensor_size, 1);

% computing distances
for k=1:sensor_size
    A = (estimated_position - [basestation_positions(k,:), 0, 0]);
    distances(k) = A * (P_hat \ A');
end

% sorting distances
[~, indexes] = sort(distances);

% extracting number of valide distances (minimum number of beacon to consider)
valide_size = size(find(distances > 0), 1) - 1;

% calculate the first index of a valide beacon
first_index  = size(indexes, 1) - valide_size;

% extract indexes
beacon_indexes = indexes(first_index : first_index + valide_size);

end

