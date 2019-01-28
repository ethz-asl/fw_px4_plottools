function RawSensorPlots(sysvector, topics, fconv_gpsalt)
% Display the raw sensor measurements

    fig1 = figure();
    fig1.Name = 'Raw Sensor Data';
    raw(1)=subplot(5,1,1);
    if topics.sensor_mag.logged
        hold on;
        plot(sysvector.sensor_mag_0.x.Time,sysvector.sensor_mag_0.x.Data);
        plot(sysvector.sensor_mag_0.y.Time,sysvector.sensor_mag_0.y.Data);
        plot(sysvector.sensor_mag_0.z.Time,sysvector.sensor_mag_0.z.Data);
        hold off
    end
    title('Magnetometers [Gauss]');
    legend('x','y','z');

    raw(2)=subplot(5,1,2);
    if topics.sensor_accel.logged
        hold on;
        plot(sysvector.sensor_accel_0.x.Time,sysvector.sensor_accel_0.x.Data);
        plot(sysvector.sensor_accel_0.y.Time,sysvector.sensor_accel_0.y.Data);
        plot(sysvector.sensor_accel_0.z.Time,sysvector.sensor_accel_0.z.Data);
        hold off
    end
    title('Accelerometers [m/s²]');
    legend('x','y','z');

    raw(3)=subplot(5,1,3);
    if topics.sensor_gyro.logged
        hold on;
        plot(sysvector.sensor_gyro_0.x.Time,sysvector.sensor_gyro_0.x.Data);
        plot(sysvector.sensor_gyro_0.y.Time,sysvector.sensor_gyro_0.y.Data);
        plot(sysvector.sensor_gyro_0.z.Time,sysvector.sensor_gyro_0.z.Data);
        hold off
    end
    title('Gyroscopes [rad/s]');
    legend('x','y','z');

    raw(4)=subplot(5,1,4);
    if topics.sensor_baro.logged
        hold on;
        plot(sysvector.sensor_baro_0.pressure.Time,sysvector.sensor_baro_0.pressure.Data);
        hold off
    end
    title('Pressure [mbar]');

    raw(5)=subplot(5,1,5);
    if topics.sensor_baro.logged
        hold on;
        plot(sysvector.sensor_baro_0.temperature.Time,sysvector.sensor_baro_0.temperature.Data);
        hold off
    end
    title('Temperature [degC]');

    linkaxes([raw(1) raw(2) raw(3) raw(4) raw(5)],'x');
    set(raw(:),'XGrid','on','YGrid','on','ZGrid','on');
    dcm_obj = datacursormode(fig1);
    set(dcm_obj,'UpdateFcn',@HighPrecisionTooltipCallback);