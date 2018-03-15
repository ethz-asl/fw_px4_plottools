function CpuLoadPlots(sysvector)
% Display the cpu load data

fig27 = figure(27);
fig27.Name = 'CPU Load';

hold on;
plot(sysvector('cpuload_0.load')); 
plot(sysvector('cpuload_0.ram_usage'));
xlabel('time [s]')
hold off;
ylim([0, 1]);
legend('CPU Load', 'RAM');
end

