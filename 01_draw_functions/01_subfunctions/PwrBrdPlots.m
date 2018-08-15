function PwrBrdPlots(sysvector, topics)
% DisplayS Battery Monitoring plots

fig1 = figure();
fig1.Name = 'Power board data';

ax(1)=subplot(5,1,1);
    %if topics.sensor_pwr_brd.logged
        %hold on;
        m_SystemPower = sysvector('sensor_pwr_brd_0.pwr_brd_mot_l_amp').Data .* sysvector('sensor_pwr_brd_0.pwr_brd_system_volt').Data + ...
            sysvector('sensor_pwr_brd_0.pwr_brd_analog_amp').Data .* sysvector('sensor_pwr_brd_0.pwr_brd_servo_volt').Data + ...
            sysvector('sensor_pwr_brd_0.pwr_brd_digital_amp').Data .* sysvector('sensor_pwr_brd_0.pwr_brd_digital_volt').Data + ...
            sysvector('sensor_pwr_brd_0.pwr_brd_ext_amp').Data .* sysvector('sensor_pwr_brd_0.pwr_brd_system_volt').Data + ...
            sysvector('sensor_pwr_brd_0.pwr_brd_aux_amp').Data .* sysvector('sensor_pwr_brd_0.pwr_brd_digital_volt').Data;
    
        plot(sysvector('sensor_pwr_brd_0.pwr_brd_mot_l_amp').Time, m_SystemPower,'LineWidth',2);
        %hold off
    %end
title('Power [W]');
ylabel('Power [W]')

end