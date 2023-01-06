function MagVsThrustPlots(sysvector, topics)
% Plot the mag norm vs thrust

if ~topics.sensor_mag.logged
    return
end

fig1 = figure();
fig1.Name = 'Magnetic Norm vs Thrust';
hold on;

legend_entries = {};
for i = 0:10
    fieldname = 'sensor_mag_' + string(i);
    if isfield(sysvector, fieldname)
        mag_data = getfield(sysvector, fieldname);
        mag_norm = sqrt(mag_data.x.Data .* mag_data.x.Data +...
            mag_data.y.Data .* mag_data.y.Data +...
            mag_data.z.Data .* mag_data.z.Data);

        plot(mag_data.x.Time, mag_norm);
        legend_entries = [legend_entries; 'Mag Norm ID ' + string(i)];
    end

end

if topics.actuator_controls_0.logged
    plot(sysvector.actuator_controls_0_0.control_3.Time, sysvector.actuator_controls_0_0.control_3.Data);
    legend_entries = [legend_entries; 'Thrust'];
end

if topics.battery_status.logged
    plot(sysvector.battery_status_0.current_filtered_a.Time, sysvector.battery_status_0.current_filtered_a.Data);
    legend_entries = [legend_entries; 'Filtered Current [A]'];
end
hold off;

legend(legend_entries);
set(gca, 'XGrid','on','YGrid','on','ZGrid','on');
dcm_obj = datacursormode(fig1);
set(dcm_obj,'UpdateFcn',@HighPrecisionTooltipCallback);
end

