function [x_hat, P_hat, h, H] = ekf_prediction(F, Q, x_hat, P_hat, z, basestations, arrayToDistance, distanceToValue, dx, dy)
%% Prediction step
% Predict the next estimated position and the estimated covariance.
%
% ARGS:
% F                 state transition model
% Q                 process noise covariance
% x_hat             estimated position
% P_hat             estimated covariance
% z                 measurements obtained
% basestations      position of the different sensors
% arrayToDistance   function to calculate the distance from an array
% distanceToValue   function to calculate the measurement given the distance
% dx                function to calculate the derivate wrt x for the measurement function (distanceToValue)
% dy                function to calculate the derivate wrt x for the measurement function (distanceToValue)


sensor_size = length(basestations);

% project the state ahead (prediction of (x,y) coordinates at t knowing state at t-1)
x_hat = F * x_hat;
% project the covariance ahead
P_hat = F * P_hat * F' + Q;

h = zeros(sensor_size,1);
for k=1:sensor_size
    if z(k) ~= 0
        h(k) = distanceToValue(arrayToDistance(x_hat(1:2)' - basestations(k,:)));
    end
end

% Build H
H = [];
for i= 1:length(z)
    if z(i) ~= 0 % linearize
        coordinate_diff = [x_hat(1) - basestations(i,1), x_hat(2) - basestations(i,2)];

        dh_dx = dx(coordinate_diff(1), coordinate_diff(2));
        dh_dy = dy(coordinate_diff(1), coordinate_diff(2));

        H = [H; dh_dx dh_dy 0 0];

    else % empty line because the basestation is too far away from the target
        H = [H; 0 0 0 0];
    end
end

end