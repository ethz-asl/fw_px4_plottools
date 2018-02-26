function DistanceSensorPlots(sysvector)
% Display the distance sensor data

fig28 = figure(28);
fig28.Name = 'Distance Sensor';

hold on;
plot(sysvector('distance_sensor_0.min_distance')); 
plot(sysvector('distance_sensor_0.max_distance')); 
plot(sysvector('distance_sensor_0.current_distance')); 
plot(sysvector('distance_sensor_0.covariance')); 
hold off;
xlabel('time [s]');
ylabel('distance [m]');
legend('Min Dist [m]', 'Max Dist [m]', 'Current Dist [m]', 'Covariance [m]');
end

