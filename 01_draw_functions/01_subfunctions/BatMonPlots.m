function BatMonPlots(sysvector, topics)
% DisplayS Battery Monitoring plots

fig1 = figure();
fig1.Name = 'Battery Monitoring Statistics';

raw(1)=subplot(5,1,1);
    if topics.bat_mon.logged
        hold on;
        plot(sysvector('bat_mon_0.voltage').Time, sysvector('bat_mon_0.voltage').Data/ 1000.0,'LineWidth',2);
        hold off
    end
title('Voltage [V]');
ylabel('voltage [V]')

raw(2)=subplot(5,1,2);
    if topics.bat_mon.logged
        hold on;
        plot(sysvector('bat_mon_0.current').Time, sysvector('bat_mon_0.current').Data/ 1000.0,'LineWidth',2);
        hold off
    end
title('Current [V]');
ylabel('current [A]')

raw(3)=subplot(5,1,3);
    if topics.bat_mon.logged
        hold on;
        plot(sysvector('bat_mon_0.stateofcharge').Time, sysvector('bat_mon_0.stateofcharge').Data,'LineWidth',2);
        hold off
    end
title('SoC [%]');
ylabel('SoC [%]')

raw(4)=subplot(5,1,4);
    if topics.bat_mon.logged
        hold on;
        plot(sysvector('bat_mon_0.temperature').Time, sysvector('bat_mon_0.temperature').Data/ 100.0,'LineWidth',2);
        hold off
    end
title('Temperature [°C]');
ylabel('temperature [°C]')

raw(5)=subplot(5,1,5);
    if topics.bat_mon.logged
        hold on;
        plot(sysvector('bat_mon_0.batterystatus').Time, sysvector('bat_mon_0.batterystatus').Data,'LineWidth',2);
        plot(sysvector('bat_mon_0.operationstatus').Time, sysvector('bat_mon_0.operationstatus').Data,'LineWidth',2);
        plot(sysvector('bat_mon_0.safetystatus').Time, sysvector('bat_mon_0.safetystatus').Data,'LineWidth',2);
        hold off
    end
title('Status registers');
legend('battery status','operation status','safety status');

linkaxes([raw(1) raw(2) raw(3) raw(4) raw(5)],'x');
set(raw(:),'XGrid','on','YGrid','on','ZGrid','on');   

fig2 = figure();
fig2.Name = 'Battery Monitoring balancing';
    if topics.bat_mon.logged
        hold on;
        plot(sysvector('bat_mon_0.cellvoltage1').Time, sysvector('bat_mon_0.cellvoltage1').Data/ 1000.0,'LineWidth',2);
        plot(sysvector('bat_mon_0.cellvoltage2').Time, sysvector('bat_mon_0.cellvoltage2').Data/ 1000.0,'LineWidth',2);
        plot(sysvector('bat_mon_0.cellvoltage3').Time, sysvector('bat_mon_0.cellvoltage3').Data/ 1000.0,'LineWidth',2);
        plot(sysvector('bat_mon_0.cellvoltage4').Time, sysvector('bat_mon_0.cellvoltage4').Data/ 1000.0,'LineWidth',2);
        plot(sysvector('bat_mon_0.cellvoltage5').Time, sysvector('bat_mon_0.cellvoltage5').Data/ 1000.0,'LineWidth',2);
        plot(sysvector('bat_mon_0.cellvoltage6').Time, sysvector('bat_mon_0.cellvoltage6').Data/ 1000.0,'LineWidth',2);
        hold off;
    end
title('Battery cells voltage');
ylabel('voltage [V]')
legend('Cell1 [V]','Cell2 [V]','Cell3 [V]','Cell4 [V]','Cell5 [V]','Cell6 [V]');
grid on;

end