function BatMonPlots(sysvector, topics)
% DisplayS Battery Monitoring plots

fig1 = figure();
fig1.Name = 'Battery Monitoring Statistics';

raw(1)=subplot(5,1,1);
    if topics.sensor_bat_mon.logged
        hold on;
        plot(sysvector.sensor_bat_mon_0.voltage.Time, sysvector.sensor_bat_mon_0.voltage.Data/ 1000.0,'LineWidth',2);
        hold off
    end
title('Voltage [V]');
ylabel('voltage [V]')

raw(end+1)=subplot(5,1,2);
    if topics.sensor_bat_mon.logged
        hold on;
        plot(sysvector.sensor_bat_mon_0.current.Time, sysvector.sensor_bat_mon_0.current.Data/ 1000.0,'LineWidth',2);
        hold off
    end
title('Current [A]');
ylabel('current [A]')

raw(end+1)=subplot(5,1,3);
    if topics.sensor_bat_mon.logged
        hold on;
        plot(sysvector.sensor_bat_mon_0.stateofcharge.Time, sysvector.sensor_bat_mon_0.stateofcharge.Data,'LineWidth',2);
        hold off
    end
title('SoC [%]');
ylabel('SoC [%]')

raw(end+1)=subplot(5,1,4);
    if topics.sensor_bat_mon.logged
        hold on;
        plot(sysvector.sensor_bat_mon_0.temperature.Time, sysvector.sensor_bat_mon_0.temperature.Data/ 100.0,'LineWidth',2);
        hold off
    end
title('Temperature [°C]');
ylabel('temperature [°C]')

raw(end+1)=subplot(5,1,5);
    if topics.sensor_bat_mon.logged
        hold on;
        plot(sysvector.sensor_bat_mon_0.batterystatus.Time, sysvector.sensor_bat_mon_0.batterystatus.Data,'LineWidth',2);
        plot(sysvector.sensor_bat_mon_0.operationstatus.Time, sysvector.sensor_bat_mon_0.operationstatus.Data,'LineWidth',2);
        plot(sysvector.sensor_bat_mon_0.safetystatus.Time, sysvector.sensor_bat_mon_0.safetystatus.Data,'LineWidth',2);
        hold off
    end
title('Status registers');
legend('battery status','operation status','safety status');

linkaxes(raw(:),'x');
set(raw(:),'XGrid','on','YGrid','on','ZGrid','on');   

fig2 = figure();
fig2.Name = 'Battery Monitoring balancing';
    if topics.sensor_bat_mon.logged
        hold on;
        plot(sysvector.sensor_bat_mon_0.cellvoltage_0.Time, sysvector.sensor_bat_mon_0.cellvoltage_0.Data/ 1000.0,'LineWidth',2);
        plot(sysvector.sensor_bat_mon_0.cellvoltage_1.Time, sysvector.sensor_bat_mon_0.cellvoltage_1.Data/ 1000.0,'LineWidth',2);
        plot(sysvector.sensor_bat_mon_0.cellvoltage_2.Time, sysvector.sensor_bat_mon_0.cellvoltage_2.Data/ 1000.0,'LineWidth',2);
        plot(sysvector.sensor_bat_mon_0.cellvoltage_3.Time, sysvector.sensor_bat_mon_0.cellvoltage_3.Data/ 1000.0,'LineWidth',2);
        plot(sysvector.sensor_bat_mon_0.cellvoltage_4.Time, sysvector.sensor_bat_mon_0.cellvoltage_4.Data/ 1000.0,'LineWidth',2);
        plot(sysvector.sensor_bat_mon_0.cellvoltage_5.Time, sysvector.sensor_bat_mon_0.cellvoltage_5.Data/ 1000.0,'LineWidth',2);
        hold off;
    end
title('Battery cells voltage');
ylabel('voltage [V]')
legend('Cell1 [V]','Cell2 [V]','Cell3 [V]','Cell4 [V]','Cell5 [V]','Cell6 [V]');
grid on;

end