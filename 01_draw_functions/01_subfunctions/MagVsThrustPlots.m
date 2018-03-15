function MagVsThrustPlots(sysvector)
% Plot the mag norm vs thrust

fig33 = figure(33);
fig33.Name = 'Magnetic Norm vs Thrust';

mag_norm = sqrt(sysvector('sensor_mag_0.x').Data .* sysvector('sensor_mag_0.x').Data +...
    sysvector('sensor_mag_0.y').Data .* sysvector('sensor_mag_0.y').Data +...
    sysvector('sensor_mag_0.z').Data .* sysvector('sensor_mag_0.z').Data);
    
hold on;
plot(sysvector('sensor_mag_0.x').Time, mag_norm);
plot(sysvector('actuator_controls_0.control_3').Time, sysvector('actuator_controls_0.control_3').Data);
hold off;

legend('Magnetic Norm', 'Thrust');
end

