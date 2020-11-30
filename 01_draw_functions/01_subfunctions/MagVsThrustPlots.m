function MagVsThrustPlots(sysvector)
% Plot the mag norm vs thrust

fig1 = figure();
fig1.Name = 'Magnetic Norm vs Thrust';

mag_norm = sqrt(sysvector.sensor_mag_0.x.Data .* sysvector.sensor_mag_0.x.Data +...
    sysvector.sensor_mag_0.y.Data .* sysvector.sensor_mag_0.y.Data +...
    sysvector.sensor_mag_0.z.Data .* sysvector.sensor_mag_0.z.Data);
    
hold on;
plot(sysvector.sensor_mag_0.x.Time, mag_norm);
plot(sysvector.actuator_controls_0_0.control_3.Time, sysvector.actuator_controls_0_0.control_3.Data);
hold off;

legend('Magnetic Norm', 'Thrust');
set(gca, 'XGrid','on','YGrid','on','ZGrid','on');
dcm_obj = datacursormode(fig1);
set(dcm_obj,'UpdateFcn',@HighPrecisionTooltipCallback);
end

