%% ************************************************************************
%  SensorPlots (nested function)
%  ************************************************************************
%  Display the sensor data from the log file. Assumes that successfully
%  sensor data was logged.

function SensorPlots(sysvector, topics, fconv_gpsalt)
    fig1 = figure();
    fig1.Name = 'Sensor Data';
    raw(1)=subplot(4,1,1);
    if (topics.vehicle_magnetometer.logged)
        hold on;
        plot(sysvector.vehicle_magnetometer_0.magnetometer_ga_0.Time,sysvector.vehicle_magnetometer_0.magnetometer_ga_0.Data);
        plot(sysvector.vehicle_magnetometer_0.magnetometer_ga_1.Time,sysvector.vehicle_magnetometer_0.magnetometer_ga_1.Data);
        plot(sysvector.vehicle_magnetometer_0.magnetometer_ga_2.Time,sysvector.vehicle_magnetometer_0.magnetometer_ga_2.Data);
        hold off
        legend('x','y','z');
    elseif (topics.sensor_mag.logged)
        hold on;
        plot(sysvector.sensor_mag_0.x.Time,sysvector.sensor_mag_0.x.Data)
        plot(sysvector.sensor_mag_0.y.Time,sysvector.sensor_mag_0.y.Data)
        plot(sysvector.sensor_mag_0.z.Time,sysvector.sensor_mag_0.z.Data)
        hold off
        legend('x','y','z')
    end
    title('Magnetometers [Gauss]');

    raw(2)=subplot(4,1,2);
    if (topics.sensor_combined.logged)
        hold on;
        plot(sysvector.sensor_combined_0.accelerometer_m_s2_0.Time,sysvector.sensor_combined_0.accelerometer_m_s2_0.Data);
        plot(sysvector.sensor_combined_0.accelerometer_m_s2_1.Time,sysvector.sensor_combined_0.accelerometer_m_s2_1.Data);
        plot(sysvector.sensor_combined_0.accelerometer_m_s2_2.Time,sysvector.sensor_combined_0.accelerometer_m_s2_2.Data);
        hold off
        legend('x','y','z');
    end
    title('Accelerometers [m/s²]');

    raw(3)=subplot(4,1,3);
    if (topics.sensor_combined.logged)
        hold on;
        plot(sysvector.sensor_combined_0.gyro_rad_0.Time,sysvector.sensor_combined_0.gyro_rad_0.Data);
        plot(sysvector.sensor_combined_0.gyro_rad_1.Time,sysvector.sensor_combined_0.gyro_rad_1.Data);
        plot(sysvector.sensor_combined_0.gyro_rad_2.Time,sysvector.sensor_combined_0.gyro_rad_2.Data);
        hold off
        legend('x','y','z');
    end
    title('Gyroscopes [rad/s]');

    raw(4)=subplot(4,1,4);
    if (topics.vehicle_air_data.logged)
        hold on;
        plot(sysvector.vehicle_air_data_0.baro_alt_meter.Time,sysvector.vehicle_air_data_0.baro_alt_meter.Data);
        if topics.vehicle_gps_position.logged
            plot(sysvector.vehicle_gps_position_0.alt.Time,double(sysvector.vehicle_gps_position_0.alt.Data)*fconv_gpsalt,'color','red');
            legend('Barometric Altitude [m]','GPS Altitude [m]');
        else
            legend('Barometric Altitude [m]');
        end
        hold off
    end
    title('Altitude above MSL [m]');

    linkaxes([raw(1) raw(2) raw(3) raw(4)],'x');
    set(raw(:),'XGrid','on','YGrid','on','ZGrid','on');
    dcm_obj = datacursormode(fig1);
    set(dcm_obj,'UpdateFcn',@HighPrecisionTooltipCallback);
end