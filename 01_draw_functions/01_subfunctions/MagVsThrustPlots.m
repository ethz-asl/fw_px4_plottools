function MagVsThrustPlots(sysvector)
% Plot the mag norm vs thrust

fig33 = figure(33);
fig33.Name = 'Magnetic Norm vs Thrust';

mag_norm = sqrt(sysvector('sensor_mag.x').Data .* sysvector('sensor_mag.x').Data +...
    sysvector('sensor_mag.y').Data .* sysvector('sensor_mag.y').Data +...
    sysvector('sensor_mag.z').Data .* sysvector('sensor_mag.z').Data);
    
hold on;
plot(sysvector('sensor_mag.x').Time, mag_norm);
plot(sysvector('actuator_controls.control_3').Time, sysvector('actuator_controls.control_3').Data);
hold off;

legend('Magnetic Norm', 'Thrust');
end

