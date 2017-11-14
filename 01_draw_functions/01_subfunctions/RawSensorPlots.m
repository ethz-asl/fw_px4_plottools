function RawSensorPlots(sysvector, topics, fconv_gpsalt)
% Display the raw sensor measurements

    fig19 = figure(19);
    fig19.Name = 'Raw Sensor Data';
    raw(1)=subplot(4,1,1);
    if topics.sensor_mag.logged
        hold on;
        plot(sysvector('sensor_mag.x').Time,sysvector('sensor_mag.x').Data);
        plot(sysvector('sensor_mag.y').Time,sysvector('sensor_mag.y').Data);
        plot(sysvector('sensor_mag.z').Time,sysvector('sensor_mag.z').Data);
        hold off
    end
    title('Magnetometers [Gauss]');
    legend('x','y','z');
    raw(2)=subplot(4,1,2);
    if topics.sensor_accel.logged
        hold on;
        plot(sysvector('sensor_accel.x').Time,sysvector('sensor_accel.x').Data);
        plot(sysvector('sensor_accel.y').Time,sysvector('sensor_accel.y').Data);
        plot(sysvector('sensor_accel.z').Time,sysvector('sensor_accel.z').Data);
        hold off
    end
    title('Accelerometers [m/s²]');
    legend('x','y','z');
    raw(3)=subplot(4,1,3);
    if topics.sensor_gyro.logged
        hold on;
        plot(sysvector('sensor_gyro.x').Time,sysvector('sensor_gyro.x').Data);
        plot(sysvector('sensor_gyro.y').Time,sysvector('sensor_gyro.y').Data);
        plot(sysvector('sensor_gyro.z').Time,sysvector('sensor_gyro.z').Data);
        hold off
    end
    title('Gyroscopes [rad/s]');
    legend('x','y','z');
    raw(4)=subplot(4,1,4);
    if topics.sensor_baro.logged
        hold on;
        plot(sysvector('sensor_combined.baro_alt').Time,sysvector('sensor_combined.baro_alt').Data);
        if topics.vehicle_gps_position.logged
            plot(sysvector('vehicle_gps_position.alt').Time,double(sysvector('vehicle_gps_position.alt').Data)*fconv_gpsalt,'color','red');
            legend('Barometric Altitude [m]','GPS Altitude [m]');
        else
            legend('Barometric Altitude [m]');
        end
        hold off
    end
    title('Altitude above MSL [m]');

    linkaxes([raw(1) raw(2) raw(3) raw(4)],'x');
    set(raw(:),'XGrid','on','YGrid','on','ZGrid','on');   