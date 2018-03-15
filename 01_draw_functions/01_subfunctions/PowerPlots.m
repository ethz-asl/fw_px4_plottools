function PowerPlots(sysvector)
% Display the power plots

fig34 = figure(34);
fig34.Name = 'Power Statistics';

hold on;
plot(sysvector('battery_status_0.voltage_v').Time, sysvector('battery_status_0.voltage_v').Data); 
plot(sysvector('battery_status_0.voltage_filtered_v').Time, sysvector('battery_status_0.voltage_filtered_v').Data); 
plot(sysvector('battery_status_0.current_a').Time, sysvector('battery_status_0.current_a').Data); 
plot(sysvector('battery_status_0.current_filtered_a').Time, sysvector('battery_status_0.current_filtered_a').Data); 
plot(sysvector('battery_status_0.discharged_mah').Time, sysvector('battery_status_0.discharged_mah').Data ./ 100); 
plot(sysvector('battery_status_0.remaining').Time, sysvector('battery_status_0.remaining').Data * 10); 
hold off;
legend('Raw Voltage [V]', 'Filtered Voltage [V]', 'Raw Current [A]', 'Filtered Current [A]',...
    'Discharged [mAh/100]', 'Remaining [0 = Empty, 10 = Full]');
end

