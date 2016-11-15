function [noised_radio_power] = generate_measurements(X, basestations, radius, var_z, arrayToDistance, distanceToValue)
%% Generate noised measurements
% Calculate the distances for each point in the path from each basestation,
% and estimate the RSSI value, the introduce some noise in the measurement.
%
% ARGS:
% X                 trajectory path
% basestations      position of the different sensors
% var_z             variance of the measuerements
% arrayToDistance   function to calculate the distance from an array
% distanceToValue   function to calculate the measurement given the distance


N = length(X);
sensor_size = length(basestations);

% noised_radio_power is a matrix [sensor_size, N]. Element [i,j] is the
% measurement of the distance between basestation i and the target at time j.
noised_radio_power = zeros(sensor_size, N);    % zeros -> creates array of all zero
radio_power = zeros(sensor_size, N);

%% Setup noised measurements

distances = zeros(sensor_size, N);   % initializing the vector with all 0

for t = 1:N
    for k = 1:sensor_size
        % caluclating distances betweeen kth basestation and the target
        distances(k,t) = arrayToDistance(X(t,:) - basestations(k,:));
        
        radio_power(k,t) = distanceToValue(distances(k,t));

        if distances(k,t) > radius
            distances(k,t) = 0;
        else
            % add noise to the rssi measure
            noised_radio_power(k,t) = radio_power(k,t) + sqrt(var_z) * randn(1);
        end
    end
end

end