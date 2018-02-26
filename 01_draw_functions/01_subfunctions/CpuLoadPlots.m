function CpuLoadPlots(sysvector)
% Display the cpu load data

fig27 = figure(27);
fig27.Name = 'CPU Load';

hold on;
plot(sysvector('cpuload.load').Time, sysvector('cpuload.load').Data); 
plot(sysvector('cpuload.ram_usage').Time, sysvector('cpuload.ram_usage').Data);
hold off;
ylim([0, 1]);
legend('CPU Load', 'RAM');
end

