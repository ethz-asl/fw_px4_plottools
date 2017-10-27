%% ************************************************************************
%  SensorPlots (nested function)
%  ************************************************************************
%  Display the sensor data from the log file. Assumes that successfully
%  sensor data was logged.

function SensorPlots(sysvector, topics, fconv_gpsalt)
    figure('Name', 'Sensor Data');
    raw(1)=subplot(4,1,1);
    hold on;
    plot(sysvector('sensor_combined.mag_0').Time,sysvector('sensor_combined.mag_0').Data);
    plot(sysvector('sensor_combined.mag_1').Time,sysvector('sensor_combined.mag_1').Data);
    plot(sysvector('sensor_combined.mag_2').Time,sysvector('sensor_combined.mag_2').Data);
    hold off
    title('Magnetometers [Gauss]');
    legend('x','y','z');
    raw(2)=subplot(4,1,2);
    hold on;
    plot(sysvector('sensor_combined.acc_0').Time,sysvector('sensor_combined.acc_0').Data);
    plot(sysvector('sensor_combined.acc_1').Time,sysvector('sensor_combined.acc_1').Data);
    plot(sysvector('sensor_combined.acc_2').Time,sysvector('sensor_combined.acc_2').Data);
    hold off
    title('Accelerometers [m/s²]');
    legend('x','y','z');
    raw(3)=subplot(4,1,3);
    hold on;
    plot(sysvector('sensor_combined.gyro_0').Time,sysvector('sensor_combined.gyro_0').Data);
    plot(sysvector('sensor_combined.gyro_1').Time,sysvector('sensor_combined.gyro_1').Data);
    plot(sysvector('sensor_combined.gyro_2').Time,sysvector('sensor_combined.gyro_2').Data);
    hold off
    title('Gyroscopes [rad/s]');
    legend('x','y','z');
    raw(4)=subplot(4,1,4);
    hold on;
    plot(sysvector('sensor_combined.baro_alt').Time,sysvector('sensor_combined.baro_alt').Data);
    if topics.vehicle_gps_position.logged
        plot(sysvector('vehicle_gps_position.alt').Time,double(sysvector('vehicle_gps_position.alt').Data)*fconv_gpsalt,'color','red');
        legend('Barometric Altitude [m]','GPS Altitude [m]');
    else
        legend('Barometric Altitude [m]');
    end
    hold off
    title('Altitude above MSL [m]');

    linkaxes([raw(1) raw(2) raw(3) raw(4)],'x');
    set(raw(:),'XGrid','on','YGrid','on','ZGrid','on');   
end