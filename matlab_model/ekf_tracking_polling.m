%% parameters

% trajectory file
% basestations: matrix of dimensions n x 2 [x_1 y_1 ; ... ; x_n y_n]
% radius: operative area of the basestations
% motion_model: 'P' or 'PV'
% measurement_model: 'euclidean' or 'rssi'
% sampling_time: how often we take the measurements from the basestations
% var_z: error variance of the basestations
% flag_weight_measurement: flag to choose to weight the selection of next measurements
% flag_print_walls: flag to choose to print or not walls
% rssi_model_coeff: array containing the coefficients of the measurement model

function [prediction, dist_err, dist_max,  RMSE_x, RMSE_y, RMSE_net] ...
        = ekf_tracking_polling(name_trajectory, ...
                                basestations, ...
                                radius, ...
                                dt, ...
                                polling_delay, ...
                                var_z,...
                                flag_weight_measurement, ...
                                flag_print_walls, ...
                                rssi_model_coeff ...
                                )
%% load the trajectory
file = load(name_trajectory);
X = file.X;     % X contains the trajectory

N = size(X, 1);

% number of sensors
sensor_size = size(basestations, 1);

plot_walls(X, basestations, sensor_size, radius, flag_print_walls);

% support functions
distanceFromCoordinates = @(x, y) sqrt(x.^2 + y.^2);

% function to convert space to RSSI
coordinatesToValue = @(x, y) rssi_model_coeff(1) * (distanceFromCoordinates(x, y)) .^3 + ...
                        rssi_model_coeff(2) * distanceFromCoordinates(x, y) .^2 + ...
                        rssi_model_coeff(3) * distanceFromCoordinates(x, y) + ...
                        rssi_model_coeff(4);

dx = @(x, y) rssi_model_coeff(1) * 1.5 * distanceFromCoordinates(x, y) * 2 * x + rssi_model_coeff(2) * 2 * x + ...
            rssi_model_coeff(3) * x * 1 / distanceFromCoordinates(x, y);
dy = @(x, y) rssi_model_coeff(1) * 1.5 * distanceFromCoordinates(x, y) * 2 * y + rssi_model_coeff(2) * 2 * y + ...
            rssi_model_coeff(3) * y * 1 / distanceFromCoordinates(x, y);
% the formula above are computed by derivating the function coordinatesToValue wrt x and y


%% process covariance : velocity acceleration model
accel_noise_mag =(0.001)^2 / dt; %% tuned by hand

Ex = [ dt^3/3  0       dt^2/2  0; ...
       0       dt^3/3  0       dt^2/2; ...
       dt^2/2  0       dt      0; ...
       0       dt^2/2  0       dt] .* accel_noise_mag; % Ex convert the process noise (stdv) into covariance matrix

%% state update matrice
F = [1 0 dt 0; ...
     0 1 0  dt; ...
     0 0 1  0; ...
     0 0 0  1];

%% Initialization estimated position
x_hat = [X(1,1); X(1,2); 0; 0 ];


%% output covariance : each sensor has its own uncertainty and it's uncorrelated with the others

Ez = eye(sensor_size) * var_z;

P = Ex; % estimate of initial position variance (covariance matrix)
prediction = x_hat';

radio_power = zeros(sensor_size, N);

% noised_radio_power is a matrix [sensor_size, N]. Element [i,j] is the
% measurement of the distance between basestation i and the target at time j.
noised_radio_power = zeros(sensor_size, N);    % zeros -> creates array of all zero


%% Setup fake data
% calculating distances for each instant from each basestation
% The distances are stored in:
%   noised_radio_power for the rssi model
%   noised_distances for the euclidean model
distances = zeros(sensor_size, N);   % initializing the vector with all 0

for t=1:N
    for k=1:sensor_size
        coordinates_diff = [(X(t,1) - basestations(k,1)), (X(t,2) - basestations(k,2))];
        
        % caluclating distances betweeen kth basestation and the target
        distances(k,t) = distanceFromCoordinates(coordinates_diff(1), coordinates_diff(2));
        
        radio_power(k,t) = coordinatesToValue(coordinates_diff(1), coordinates_diff(2));

        if distances(k,t) > radius
            distances(k,t) = 0;
        else
            % add noise to the rssi measure
            noised_radio_power(k,t) = radio_power(k,t) + sqrt(var_z) * randn(1);
        end
    end
end

number_est = 1;
dist_max = 0;

%% Kalman filter

for t=1:N
    %% prediction step
    
    % project the state ahead (prediction of (x,y) coordinates at t knowing state at t-1)
    x_hat = F * x_hat;
    % project the covariance ahead
    P_hat = F * P * F' + Ex;
    
    % Extract measurement from noised radio power from the three basestations
    [z, t] = selective_extraction(noised_radio_power, x_hat', P_hat, basestations, ...
                                    polling_delay, t, flag_weight_measurement);
    
    h = zeros(sensor_size,1);
    for k=1:sensor_size
        if z(k) ~= 0
            h(k) = coordinatesToValue((x_hat(1) - basestations(k,1)), (x_hat(2) - basestations(k,2)));
        end
    end
    
    
    %% building H matrix
    %H is the measurement equation
    H = [];
    active_sensors = 0;
    for i=1:size(z,1)
        if z(i) ~= 0 % si linearizza e si calcola nella predizione precedente
            coordinate_diff = [x_hat(1) - basestations(i,1), x_hat(2) - basestations(i,2)];
            
            dh_dx = dx(coordinate_diff(1), coordinate_diff(2));
            dh_dy = dy(coordinate_diff(1), coordinate_diff(2));
            active_sensors = active_sensors + 1;
            
            H = [H;  dh_dx dh_dy 0 0 ];
            
        else % empty line because the basestation is too far away from the target
            H = [H; 0 0 0 0];
        end
    end
    if active_sensors<2
        disp('Attenzione sensori rilevati inferiori a 2');
    end
    
    %% correction step
    
    % compute the kalman gain
    K = P_hat * H' / (H * P_hat * H' + Ez);
    
    % update the state estimate with measurement z(t)
    x_hat = x_hat + K * (z - h);
    % update the error covariance
    
    P = (eye(4)- K * H) * P_hat;
    
    prediction = [prediction; x_hat'];
    
    %% compute distance error
    x_err(number_est) = X(t,1) - x_hat(1);
    y_err(number_est) = X(t,2) - x_hat(2);
    
    dist(number_est) = sqrt(x_err(number_est).^2 + y_err(number_est).^2);
    
    if dist(number_est) > dist_max
        dist_max = dist(number_est);
    end
    
    number_est = number_est+1;
end


dist_err = sum(dist) / number_est;
RMSE_x = sqrt(sum(x_err.^2) / number_est);
RMSE_y = sqrt(sum(y_err.^2) / number_est);
RMSE_net = sqrt(RMSE_x.^2 + RMSE_y.^2);
