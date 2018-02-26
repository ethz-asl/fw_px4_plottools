function PowerPlots(sysvector)
% Display the power plots

fig34 = figure(34);
fig34.Name = 'Power Statistics';

hold on;
plot(sysvector('battery_status.v').Time, sysvector('battery_status.v').Data); 
plot(sysvector('battery_status.v_filtered').Time, sysvector('battery_status.v_filtered').Data); 
plot(sysvector('battery_status.i').Time, sysvector('battery_status.i').Data); 
plot(sysvector('battery_status.i_filtered').Time, sysvector('battery_status.i_filtered').Data); 
plot(sysvector('battery_status.discharged_mah').Time, sysvector('battery_status.discharged_mah').Data ./ 100); 
plot(sysvector('battery_status.remaining').Time, sysvector('battery_status.remaining').Data * 10); 
hold off;
legend('Raw Voltage [V]', 'Filtered Voltage [V]', 'Raw Current [A]', 'Filtered Current [A]',...
    'Discharged [mAh/100]', 'Remaining [0 = Empty, 10 = Full]');
end

