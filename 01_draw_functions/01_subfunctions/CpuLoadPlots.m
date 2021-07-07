function CpuLoadPlots(sysvector, topics)
% Display the cpu load data

if ~topics.cpuload.logged
   return
end

fig1 = figure();
fig1.Name = 'CPU Load';

hold on;
plot(sysvector.cpuload_0.load); 
plot(sysvector.cpuload_0.ram_usage);
xlabel('time [s]')
hold off;
ylim([0, 1]);
legend('CPU Load', 'RAM');

set(gca, 'XGrid','on','YGrid','on','ZGrid','on');
end

