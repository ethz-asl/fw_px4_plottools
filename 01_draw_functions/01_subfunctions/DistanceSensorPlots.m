function DistanceSensorPlots(sysvector)
% Display the distance sensor data

fig28 = figure(28);
fig28.Name = 'Distance Sensor';

hold on;
plot(sysvector('distance_sensor.min_dist').Time, sysvector('distance_sensor.min_dist').Data); 
plot(sysvector('distance_sensor.max_dist').Time, sysvector('distance_sensor.max_dist').Data); 
plot(sysvector('distance_sensor.current_dist').Time, sysvector('distance_sensor.current_dist').Data); 
plot(sysvector('distance_sensor.covariance').Time, sysvector('distance_sensor.covariance').Data); 
hold off;
legend('Min Dist [m]', 'Max Dist [m]', 'Current Dist [m]', 'Covariance [m]');
end

