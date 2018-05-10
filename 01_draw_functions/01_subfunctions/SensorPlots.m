%% ************************************************************************
%  SensorPlots (nested function)
%  ************************************************************************
%  Display the sensor data from the log file. Assumes that successfully
%  sensor data was logged.

function SensorPlots(sysvector, topics, fconv_gpsalt)
    fig4 = figure(4);
    fig4.Name = 'Sensor Data';
    raw(1)=subplot(4,1,1);
    hold on;
    plot(sysvector('sensor_mag_0.x').Time,sysvector('sensor_mag_0.x').Data);
    plot(sysvector('sensor_mag_0.y').Time,sysvector('sensor_mag_0.y').Data);
    plot(sysvector('sensor_mag_0.z').Time,sysvector('sensor_mag_0.z').Data);
    hold off
    title('Magnetometers [Gauss]');
    legend('x','y','z');
    raw(2)=subplot(4,1,2);
    hold on;
    plot(sysvector('sensor_combined_0.accelerometer_m_s2_0').Time,sysvector('sensor_combined_0.accelerometer_m_s2_0').Data);
    plot(sysvector('sensor_combined_0.accelerometer_m_s2_1').Time,sysvector('sensor_combined_0.accelerometer_m_s2_1').Data);
    plot(sysvector('sensor_combined_0.accelerometer_m_s2_2').Time,sysvector('sensor_combined_0.accelerometer_m_s2_2').Data);
    hold off
    title('Accelerometers [m/s²]');
    legend('x','y','z');
    raw(3)=subplot(4,1,3);
    hold on;
    plot(sysvector('sensor_combined_0.gyro_rad_0').Time,sysvector('sensor_combined_0.gyro_rad_0').Data);
    plot(sysvector('sensor_combined_0.gyro_rad_1').Time,sysvector('sensor_combined_0.gyro_rad_1').Data);
    plot(sysvector('sensor_combined_0.gyro_rad_2').Time,sysvector('sensor_combined_0.gyro_rad_2').Data);
    hold off
    title('Gyroscopes [rad/s]');
    legend('x','y','z');
    raw(4)=subplot(4,1,4);
    hold on;
    plot(sysvector('vehicle_air_data.baro_alt_meter').Time,sysvector('vehicle_air_data.baro_alt_meter').Data);
    if topics.vehicle_gps_position.logged
        plot(sysvector('vehicle_gps_position_0.alt').Time,double(sysvector('vehicle_gps_position_0.alt').Data)*fconv_gpsalt,'color','red');
        legend('Barometric Altitude [m]','GPS Altitude [m]');
    else
        legend('Barometric Altitude [m]');
    end
    hold off
    title('Altitude above MSL [m]');

    linkaxes([raw(1) raw(2) raw(3) raw(4)],'x');
    set(raw(:),'XGrid','on','YGrid','on','ZGrid','on');   
end