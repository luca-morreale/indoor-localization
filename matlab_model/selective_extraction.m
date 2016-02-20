function [ z, time ] = selective_extraction( X, position, beacons_position, polling_delay, t, radius )

% Starting from a time 't' this function polls all
% sensor with a small delay between each request.
% It just extract all position from 'X', incrementing
% at each iteration the time by polling_delay

time = t;
[sensor_size, N] = size(X);

% extract the indexes of beacons, checking the distances 
indexes = position_selector(position, beacons_position, radius, 3);

z = zeros(sensor_size, 1);

for sensor=1:size(indexes, 1)
    % time taking into account the delay of polling
    % NB X is an array and can not be accessed using real values
    %       so time should be integer and also polling_delay
    time = t + (sensor-1) * polling_delay;
    i = indexes(sensor);

    if N < time
        time = N;
    end
    z(i) = X(i, time);
end

end