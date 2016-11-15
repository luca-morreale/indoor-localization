function [x_hat, P_hat] = ekf_correction(x_hat, P_hat, R, z, h, H)
%% Correction step
% Correct the prediction of estimated position and the estimated covariance.
%
% ARGS:
% x_hat             estimated position
% P_hat             estimated covariance
% R                 observation covariance
% z                 measurements obtained
% h                 predicted measurement
% H                 observation model


% measurement residual
y = z - h;

% residual covariance
S = H * P_hat * H' + R;

% kalman gain
K = P_hat * H' / S;

% update the state estimate with measurement z(t)
x_hat = x_hat + K * y;
% update the error covariance

P_hat = (eye(4)- K * H) * P_hat;

end