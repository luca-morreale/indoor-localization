function [prediction_sequence, dist_err, dist_max,  RMSE_x, RMSE_y, RMSE_net] ...
                                            = ekf_tracking_polling_2(name_trajectory, ...
                                                                    basestations, ...
                                                                    radius, ...
                                                                    dt, ...
                                                                    polling_delay, ...
                                                                    var_z, ...
                                                                    rssi_model_coeff ...
                                                                    )
%% load the trajectory
file = load(name_trajectory);
X = file.X;     % X contains the trajectory

%% Setup variables
% number of points in the trajectory
N = size(X, 1);

% number of sensors
sensor_size = size(basestations, 1);

% support functions
arrayToDistance = @(array) sqrt(array * array');
distanceFromCoordinates = @(x, y) sqrt(x.^2 + y.^2);

% function to convert distance to RSSI
distanceToValue = @(distance) rssi_model_coeff(1) * (distance) .^3 + ...
                        rssi_model_coeff(2) * distance .^2 + ...
                        rssi_model_coeff(3) * distance + ...
                        rssi_model_coeff(4);

dx = @(x, y) rssi_model_coeff(1) * 1.5 * distanceFromCoordinates(x, y) * 2 * x + rssi_model_coeff(2) * 2 * x + ...
            rssi_model_coeff(3) * x * 1 / distanceFromCoordinates(x, y);
dy = @(x, y) rssi_model_coeff(1) * 1.5 * distanceFromCoordinates(x, y) * 2 * y + rssi_model_coeff(2) * 2 * y + ...
            rssi_model_coeff(3) * y * 1 / distanceFromCoordinates(x, y);
% the formula above are computed by derivating the function coordinatesToValue wrt x and y


% Process covariance : velocity acceleration model
accel_noise_mag =(0.001)^2 / dt; %% tuned by hand

% Process noise covariance matrix
Q = [ dt^3/3  0       dt^2/2  0; ...
       0       dt^3/3  0       dt^2/2; ...
       dt^2/2  0       dt      0; ...
       0       dt^2/2  0       dt] .* accel_noise_mag;

% Estimated initial covariance matrix
P_hat = Q;
   
% State transition model matrix
F = [1 0 dt 0; ...
     0 1 0  dt; ...
     0 0 1  0; ...
     0 0 0  1];

% Initialization estimated position
x_hat = [X(1,1); X(1,2); 0; 0 ];

% Observation covariance matrix
R = eye(sensor_size) * var_z;


prediction_sequence = x_hat';
measurements_list = generate_measurements(X, basestations, radius, var_z, arrayToDistance, distanceToValue);


%% Plot some 'fake' walls in the map, to make all more realistic 
plot_walls(X, basestations, sensor_size, radius);

%% EKF
number_est = 1;
dist_max = 0;

for t = 1:N
    
    % Extract measurement from noised radio power from the three basestations
    [measurements, t] = selective_extraction(measurements_list, x_hat', P_hat, ...
                                            basestations, polling_delay, t);
    
    % perform prediction and correction one measurement at a time
    for ii = 1:sensor_size
        % z must be a list of measurements!
        z = zeros(sensor_size, 1);
        z(ii) = measurements(ii);
        
        
        [x_hat, P_hat, h, H] = ekf_prediction(F, Q, x_hat, P_hat, z, basestations, arrayToDistance, distanceToValue, dx, dy);
        
        [x_hat, P_hat] = ekf_correction(x_hat, P_hat, R, z, h, H);
        
        prediction_sequence = [prediction_sequence; x_hat'];

        %% compute distance error
        x_err(number_est) = X(t,1) - x_hat(1);
        y_err(number_est) = X(t,2) - x_hat(2);

        dist(number_est) = sqrt(x_err(number_est).^2 + y_err(number_est).^2);

        if dist(number_est) > dist_max
            dist_max = dist(number_est);
        end
        
        number_est = number_est + 1;

    end

end

dist_err = sum(dist) / number_est;
RMSE_x = sqrt(sum(x_err.^2) / number_est);
RMSE_y = sqrt(sum(y_err.^2) / number_est);
RMSE_net = sqrt(RMSE_x.^2 + RMSE_y.^2);

end