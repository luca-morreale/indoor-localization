function [ position ] = initilize_position( beacon_positions, real_position )
%INITILIZE_POSITION Summary of this function goes here
%   Detailed explanation goes here


k = length(beacon_positions);
old_distance = inf;

for ii = 1:k
    distance = real_position - beacon_positions(ii)' * (real_position - beacon_positions(ii))
    if distance < old_distance
        old_distance = distance;
        position = beacon_positions(ii,:);
    end
end

end

