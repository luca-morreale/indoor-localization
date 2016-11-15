function [] = plot_walls(X, beacons, sensor_size, radius)
%% plot the trajectory and sensors position
figure;
plot(X(:,1),X(:,2));
hold on;
plot(beacons(:,1),beacons(:,2), 'x');

t = linspace(0,2*pi);
for j=1:sensor_size
    plot(radius*cos(t)+beacons(j,1),radius*sin(t)+beacons(j,2),'--');
end


%% plot borders (identified manually)
border= [0.25 0; 10.5 0; 10.5 2.5 ; 14.5 2.5; 14.5 0; 25 0; 25 10; 0.25 10; 0.25 0];
wall = [0.25 3; 8 3; 8 6.5; 8 7.5; 8 10];
wall_2 = [25 4.5; 21 4.5; 19 4.5; 15 4.5; 15 6.5; 15 7.5 ;15 8 ;17 8; 17 10];

plot(border(:,1), border(:,2),'black');
hold on;
plot(wall(1:3,1), wall(1:3,2),'black');
plot(wall(4:end,1), wall(4:end,2),'black');
plot(wall_2(1:2,1), wall_2(1:2,2),'black');
plot(wall_2(3:5,1), wall_2(3:5,2),'black');
plot(wall_2(6:end,1), wall_2(6:end,2),'black'); 



end

