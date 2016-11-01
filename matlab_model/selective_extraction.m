function [ z, time ] = selective_extraction( track, estimated_position, P_hat, ...
                                                basestation_positions, polling_delay, t, ...
                                                flag_weight_measurement)
% Starting from a time 't' this function polls all
% sensor with a small delay between each request.
% It just extract all position from 'track', incrementing
% at each iteration the time by polling_delay

time = t;
[sensor_size, N] = size(track);

% extract the indexes of beacons, checking the distances 
indexes = position_selector(estimated_position, P_hat, basestation_positions, 3, flag_weight_measurement);

z = zeros(sensor_size, 1);

for sensor=1:size(indexes, 1)
    % time taking into account the delay of polling
    % NB track is an array and can not be accessed using real values
    %       so time should be integer and also polling_delay
    time = t + (sensor - 1) * polling_delay;
    % i = sensor ID
    i = indexes(sensor);

    if N < time
        time = N;
    end
    % return
    z(i) = track(i, time);
end

end