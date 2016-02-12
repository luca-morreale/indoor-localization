%% main script with parameters
clear all;
close all;
clc;

%% define the positions of the sensors
beacons_position = [7.5,3.5;   ...
                     15,5;      ...
                     20,3.5;    ...
                     20,7.5;    ...
                     7.5,7.5;   ...
                     10,10;     ...
                     10,2;      ...
                     17.5,10;   ...
                     17.5,2;    ...
                    ];

beacons_radius = 7;
sampling_time = 1;    % distance between each "polling cycle"
var_z = 0.1;

motion_model = 'PV';


%name_trajectory = 'easy2.mat';
name_trajectory = 'hard2.mat';

polling_delay = 30;

[prediction, dist_err, dist_max, RMSE_x, RMSE_y, RMSE_net] = ekf_tracking_polling( name_trajectory, ...
                                                                             beacons_position, ...
                                                                             beacons_radius, ...
                                                                             sampling_time, ...
                                                                             polling_delay, ...
                                                                             var_z ...
                                                                            );

disp(['Distance Error  Avg: ',num2str(dist_err)]);
disp(['Distance Error Max : ',num2str(dist_max)]);
disp(['RMSE_x : ',num2str(RMSE_x)]);
disp(['RMSE_y : ',num2str(RMSE_y)]);
disp(['RMSE_net : ',num2str(RMSE_net)]);

if (strcmp(measurement_model, 'euclidean'))
    plot(prediction(:,1),prediction(:,2),'Color','Green');
elseif (strcmp(measurement_model, 'rssi'))
    plot(prediction(:,1),prediction(:,2),'Color','Red');
end

xlabel('x coordinate [m]');
ylabel('y coordinate [m]');    