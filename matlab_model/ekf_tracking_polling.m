%% parameters

% trajectory file
% beacons: matrix of dimensions n_beacons x 2 [x_1 y_1 ; ... ; x_n y_n]
% radius: operative area of the beacons
% motion_model: 'P' or 'PV'
% measurement_model: 'euclidean' or 'rssi'
% sampling_time: how often we take the measurements from the beacons
% var_z: error variance of the beacons
%

function [prediction, dist_err, dist_max,  RMSE_x, RMSE_y, RMSE_net] = ekf_tracking_polling(name_trajectory, ...
                                                                beacons, ...
                                                                radius, ...
                                                                sampling_time, ...
                                                                polling_delay, ...
                                                                var_z,...
                                                                measurement_weighted, ...
                                                                print_beacons_range ...
                                                                )
%% load the trajectory
file = load(name_trajectory);
X = file.X;     % X contains the trajectory

N = size(X,1);

% define polling query of sensors
dt = sampling_time;
%dt = 25*10.^(-3);
% number of sensors
sensor_size = size(beacons,1);

% dimension of the states : 2D motion x,y
n = 2;

plot_walls(X, beacons, sensor_size, radius, print_beacons_range);

% function to convert space to RSSI
spaceToValue = @(x,y) (-0.1416)*(sqrt(x.^2 + y.^2)).^3 + 2.311 * (x.^2 + y.^2) + -14.76*sqrt(x.^2 + y.^2) + 36.79;
% function derived from measurements in centimeters
% spaceToValue = @(x,y) (-1.416e-07)*(sqrt(x.^2 + y.^2)).^3 + 0.0002311*(x.^2 + y.^2) + -0.1476*sqrt(x.^2 + y.^2) + 36.79;
% function derived using all data
%spaceToValue = @(x)  (-1.104e-07)*x.^3 + 0.0002158*x.^2 + -0.1515*x + 35.83;



%% process covariance : velocity acceleration model
% accel_noise_mag = .001; % process noise: the variability in how fast the target is speeding up
%                           (stdv of acceleration: meters/sec^2)
%accel_noise_mag =(delta_v_mean)^2 / dt; % NOT WORKING AS EXPECTED PROBABLY BECAUSE OF DATASET
accel_noise_mag =(0.001)^2 / dt; %% tuned by hand

Ex = [  dt^3/3 0 dt^2/2 0; ...
    0 dt^3/3 0 dt^2/2; ...
    dt^2/2 0 dt 0; ...
    0 dt^2/2 0 dt] .* accel_noise_mag; % Ex convert the process noise (stdv) into covariance matrix

%% [state transition (state + velocity)] + [input control (acceleration)]
F = [1 0 dt 0; ...
    0 1 0 dt; ...
    0 0 1 0; ...
    0 0 0 1]; %state update matrice

%% Initialization
x_hat = [X(1,1); X(1,2); 0; 0 ];



%% output covariance : each sensor has its own uncertainty and it's uncorrelated with the others

Ez = eye(sensor_size)*var_z;

P = Ex; % estimate of initial position variance (covariance matrix)
prediction = x_hat';

radio_power = zeros(sensor_size,N);
% noised_radio_power is a matrix [sensor_size, N]. Element [i,j] is the
% measurement of the distance between beacon i and the target at time j.
noised_radio_power = zeros(sensor_size,N);    % zeros -> creates array of all zero


%% Kalman filter
% calculating distances for each instant from each beacon
% The distances are stored in:
%   noised_radio_power for the rssi model
%   noised_distances for the euclidean model
distances = zeros(sensor_size,N);   % initializing the vector with all 0

for t=1:N
    
    for k=1:sensor_size
        % caluclating distances betweeen kth beacon and the target
        distances(k,t) = sqrt((X(t,1)-beacons(k,1)).^2 + (X(t,2)-beacons(k,2)).^2);
        
        %radio_power(k,t) = Pd_0 - 5*l*log10(distances(k,t));
        radio_power(k,t) = spaceToValue((X(t,1)-beacons(k,1)), (X(t,2)-beacons(k,2)));

        if distances(k,t) > radius
            distances(k,t) = 0;
        else
            % add noise to the rssi measure
            noised_radio_power(k,t) = radio_power(k,t) + sqrt(var_z)*randn(1);
        end
    end
end

number_est = 1;
dist_max = 0;

for t=1:N
    %% prediction step
    
    % project the state ahead (prediction of (x,y) coordinates at t knowing state at t-1)
    x_hat = F * x_hat;
    % project the covariance ahead
    P_hat = F*P*F' + Ex;
    
    
    % Extract measurement from noised radio power from the three beacons
    % that are supposed to be closest to the target
    % z is a column vector containing up to 3 measurement
    [z, t] = selective_extraction(noised_radio_power, x_hat', beacons, polling_delay, t, radius, measurement_weighted, P_hat);
    
    
    h = zeros(sensor_size,1);
    for k=1:sensor_size
        if z(k) ~= 0
            %h(k) = Pd_0 - 5*l*log10(h(k));
            h(k) = spaceToValue((x_hat(1) - beacons(k,1)), (x_hat(2) - beacons(k,2)));
        end
    end
    
    
    %% building H matrix
    %H is the measurement equation
    H = [];
    active_sensors = 0;
    for i=1:size(z,1)
        if z(i) ~= 0 % si linearizza e si calcola nella predizione precedente
            
            dx = @(x,y) (-0.1416)*1.5*(sqrt(x.^2 + y.^2)) * 2*x + 2.311*2*x + -14.76* x * 1/sqrt(x.^2 + y.^2);
            dy = @(x,y) (-0.1416)*1.5*(sqrt(x.^2 + y.^2)) * 2*y + 2.311*2*y + -14.76* y * 1/sqrt(x.^2 + y.^2);
            % the formula above are computed by derivating the function spaceToValue wrt x and y
            
            dh_dx = dx(x_hat(1)-beacons(i,1), x_hat(2)-beacons(i,2));
            dh_dy = dy(x_hat(1)-beacons(i,1), x_hat(2)-beacons(i,2));
            active_sensors = active_sensors + 1;
            
            H = [H;  dh_dx dh_dy 0 0 ];
            
        else % empty line because the beacon is too far away from the target
            H = [H; 0 0 0 0];
        end
    end
    if active_sensors<2
        disp('Attenzione sensori rilevati inferiori a 2');
    end
    
    %% correction step
    
    % compute the kalman gain
    K = P_hat*H' * inv(H*P_hat*H' + Ez);
    
    % update the state estimate with measurement z(t)
    x_hat = x_hat + K*(z-h);
    % update the error covariance
    
    P = (eye(4)-K*H)*P_hat;
    
    prediction = [prediction; x_hat'];
    
    %% compute distance error
    x_err(number_est) = X(t,1) - x_hat(1);
    y_err(number_est) = X(t,2) - x_hat(2);
    
    dist(number_est) = sqrt(x_err(number_est).^2 + y_err(number_est).^2);
    
    if dist(number_est) > dist_max
        dist_max = dist(number_est);
        pos_dist_max = number_est;
    end
    
    number_est = number_est+1;
    
end

%debug
%prediction

dist_err = sum(dist) / number_est;
RMSE_x = sqrt(sum(x_err.^2)/number_est);
RMSE_y = sqrt(sum(y_err.^2)/number_est);
RMSE_net = sqrt(RMSE_x.^2 + RMSE_y.^2);