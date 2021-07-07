function MpptPlots(sysvector, topics)
% DisplayS MPPT plots

if ~topics.sensor_mppt.logged
    return
end

fig1 = figure();
fig1.Name = 'MPPT data';

raw(1)=subplot(4,1,1);
hold on;
plot(sysvector.sensor_mppt_0.mppt_amp_0.Time, sysvector.sensor_mppt_0.mppt_amp_0.Data,'LineWidth',2);
plot(sysvector.sensor_mppt_0.mppt_amp_1.Time, sysvector.sensor_mppt_0.mppt_amp_1.Data,'LineWidth',2);
plot(sysvector.sensor_mppt_0.mppt_amp_2.Time, sysvector.sensor_mppt_0.mppt_amp_2.Data,'LineWidth',2);
hold off
title('Current [A]');
ylabel('current [A]')
legend('MPPT1','MPPT2','MPPT3');
grid on;

raw(end+1)=subplot(4,1,2);
hold on;
plot(sysvector.sensor_mppt_0.mppt_volt_0.Time, sysvector.sensor_mppt_0.mppt_volt_0.Data,'LineWidth',2);
plot(sysvector.sensor_mppt_0.mppt_volt_1.Time, sysvector.sensor_mppt_0.mppt_volt_1.Data,'LineWidth',2);
plot(sysvector.sensor_mppt_0.mppt_volt_2.Time, sysvector.sensor_mppt_0.mppt_volt_2.Data,'LineWidth',2);
hold off
title('Voltage [V]');
ylabel('voltage [V]')
legend('MPPT1','MPPT2','MPPT3');

raw(end+1)=subplot(4,1,3);
hold on;
plot(sysvector.sensor_mppt_0.mppt_pwm_0.Time, sysvector.sensor_mppt_0.mppt_pwm_0.Data,'LineWidth',2);
plot(sysvector.sensor_mppt_0.mppt_pwm_1.Time, sysvector.sensor_mppt_0.mppt_pwm_1.Data,'LineWidth',2);
plot(sysvector.sensor_mppt_0.mppt_pwm_2.Time, sysvector.sensor_mppt_0.mppt_pwm_2.Data,'LineWidth',2);
hold off
title('PWM');
ylabel('pwm')
legend('MPPT1','MPPT2','MPPT3');

raw(end+1)=subplot(4,1,4);
hold on;
plot(sysvector.sensor_mppt_0.mppt_status_0.Time, sysvector.sensor_mppt_0.mppt_status_0.Data,'LineWidth',2);
plot(sysvector.sensor_mppt_0.mppt_status_1.Time, sysvector.sensor_mppt_0.mppt_status_1.Data,'LineWidth',2);
plot(sysvector.sensor_mppt_0.mppt_status_2.Time, sysvector.sensor_mppt_0.mppt_status_2.Data,'LineWidth',2);
hold off
title('Status');
ylabel('status')
legend('MPPT1','MPPT2','MPPT3');

linkaxes([raw(:)],'x');
set(raw(:),'XGrid','on','YGrid','on','ZGrid','on');

end
