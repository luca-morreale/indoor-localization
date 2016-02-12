function [ z ] = extract_distance_with_polling_delay( X, polling_delay, sensor_size, t )

% Starting from a time 't' this function polls all
% sensor with a small delay between each request.
% It just extract all position from 'X', incrementing
% at each iteration the time by polling_delay

z = zeros(sensor_size);

for sensor=1:sensor_size
    % time taking into account the delay of polling
    % NB X is an array and can not be accessed using real values
    %       so time should be integer and also polling_delay
    time = t + (sensor-1) * polling_delay;
    z(sensor) = X(sensor,time);
end

end
