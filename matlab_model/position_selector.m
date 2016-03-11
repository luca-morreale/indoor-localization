function [ beacon_indexes ] = poistion_selector( position, beacons_position, radius, qt, measurement_weighted, P_hat)
% This function computes the distances bewtween each beacons and the 
% given position, and then extract the index of first _qt_ nearer beacons 

[sensor_size, N] = size(beacons_position);
distances = zeros(sensor_size, 1);

% computing distances
for k=1:sensor_size
%    distances(k) = sqrt((position(1)-beacons_position(k,1)).^2 + ...
%                        (position(2)-beacons_position(k,2)).^2);

    if not(measurement_weighted)
        distances(k) = (position(1:2)-beacons_position(k,1:2)) * (position(1:2)-beacons_position(k,1:2))';
    else
        A = (position-[beacons_position(k,:), 0, 0]);
        distances(k) = A * inv(P_hat) * A';
    end
    
    distances
   % chi quadro 2 gradi libertà al 95%
   % dof = 2;
   % y = chipdf(radius*radius, dof)
   % or
   % (position(1:2) - beacons_position(k,1:2)).^2 /
   %     beacons_position(k,1:2);
   
    if distances(k) > radius*radius
        distances(k) = 0;
    end
    
end

% sorting distances
[sorted_distances, indexes] = sort(distances);

% extracting number of valide distances
valide_size = size(find(distances > 0), 1) - 1;

% calculate the minimum number of beacon to consider
offset = min(valide_size, qt-1);
% calculate the first index of a valide beacon
first_index  = size(indexes,1)-valide_size;

% extract indexes
beacon_indexes = indexes(first_index : first_index + offset);

end

