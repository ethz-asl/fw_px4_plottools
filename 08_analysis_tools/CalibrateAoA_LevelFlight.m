% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
% Calibrate AoA in Level Flight
% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

% !!-- This script must be run *after running the logconv.m script. --!!

% Assuming in steady level flight with constant wind the AoA should be
% equal to the pitch angle

%% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
% Plot wind reconstructions from (uncalibrated) true airspeed and ground
% speed measurements to approximate initial guess of NE wind components and
% choose a start and end time of appropriate calibration data. Calibration
% data should satisfy the following criteria:
% - in air at nominal airspeed
% - constant altitude
% - loitering with minimal bank for at least 3 cycles (hint: use stabilized
%   modes for collecting this data)
% - wind speeds are assumed to be constant within the seleted data

% start and end times (modify these)
t_st_cal = 740;
t_ed_cal = 770;

% ! START do not modify ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
clc;
if (topics.airspeed.logged && topics.vehicle_gps_position.logged && ...
    topics.vehicle_attitude.logged && topics.airflow_aoa.logged)

    % synchronise the data
    dt = 0.05;
    min_time = max([t_st_cal, ...
        sysvector.vehicle_gps_position_0.vel_d_m_s.Time(1),...
        sysvector.airspeed_0.true_airspeed_m_s.Time(1),...
        sysvector.airflow_aoa_0.aoa_rad.Time(1),...
        sysvector.airflow_slip_0.slip_rad.Time(1),...
        sysvector.vehicle_attitude_0.q_0.Time(1)]);
    max_time = min([t_ed_cal, ...
        sysvector.vehicle_gps_position_0.vel_d_m_s.Time(end),...
        sysvector.airspeed_0.true_airspeed_m_s.Time(end),...
        sysvector.airflow_aoa_0.aoa_rad.Time(end),...
        sysvector.airflow_slip_0.slip_rad.Time(end),...
        sysvector.vehicle_attitude_0.q_0.Time(end)]);
    if (topics.vehicle_local_position.logged)
        min_time = max([min_time, sysvector.vehicle_local_position_0.x.Time(1)]);
        max_time = min([max_time, sysvector.vehicle_local_position_0.x.Time(end)]);
    end
    time_resampled = min_time:dt:max_time;

    % resample the data
    vel_d = resample(sysvector.vehicle_gps_position_0.vel_d_m_s, time_resampled);
    airspeed = resample(sysvector.airspeed_0.true_airspeed_m_s, time_resampled);
    aoa = resample(sysvector.airflow_aoa_0.aoa_rad, time_resampled);
    q_0 = resample(sysvector.vehicle_attitude_0.q_0, time_resampled);
    q_1 = resample(sysvector.vehicle_attitude_0.q_1, time_resampled);
    q_2 = resample(sysvector.vehicle_attitude_0.q_2, time_resampled);
    q_3 = resample(sysvector.vehicle_attitude_0.q_3, time_resampled);
    num_plots = 4;
    if (topics.vehicle_local_position.logged)
        num_plots = 5;
        pos_x = resample(sysvector.vehicle_local_position_0.x, time_resampled);
        pos_y = resample(sysvector.vehicle_local_position_0.y, time_resampled);
        pos_z = resample(sysvector.vehicle_local_position_0.z, time_resampled);
    end
    [pitch, roll, yaw] = QuaternionToEuler(q_0, q_1, q_2, q_3);
    
    % plot the data
    if (topics.vehicle_local_position.logged)
        plot3(pos_x.Data,pos_y.Data,pos_z.Data,'k.');
        axis equal
        hold all
        text(pos_x.Data(1),pos_y.Data(1),pos_z.Data(1),'t_{min}');
        text(pos_x.Data(end),pos_y.Data(end),pos_z.Data(end),'t_{max}');
        di = numel(pos_x.Data);
        distep = floor(di/20);
        for i=1:20
            text(pos_x.Data(i*distep), ...
                pos_y.Data(i*distep), ...
                pos_z.Data(i*distep), ...
                num2str(pos_x.Time(i*distep,1), '%.1f'));
        end
        title('Flight Path');
        xlabel('delta-Longitude [m]');
        ylabel('delta-Latitude [m]');
        zlabel('Altitude above MSL [m]');
        grid on
    end
    
    figure('color', 'w', 'name', 'Steady Level Flight Calibration');
    wind_plot(1) = subplot(num_plots,1,1); hold on; grid on; box on;
    plot(time_resampled, -vel_d.Data);
    ylabel('Vertical Velocity [m/s]');
    
    wind_plot(2) = subplot(num_plots,1,2); hold on; grid on; box on;
    plot(time_resampled, airspeed.Data);
    ylabel('Airspeed [m/s]');

    wind_plot(3) = subplot(num_plots,1,3); hold on; grid on; box on;
    plot(time_resampled, rad2deg(roll.Data));
    plot(time_resampled, rad2deg(pitch.Data));
    plot(time_resampled, rad2deg(yaw.Data));
    legend('roll', 'pitch', 'yaw');
    ylabel('Attitude [deg]');
    
    wind_plot(4) = subplot(num_plots,1,4); hold on; grid on; box on;
    plot(time_resampled, rad2deg(aoa.Data));
    ylabel('AoA [deg]');
    
    if (topics.vehicle_local_position.logged)
        wind_plot(5) = subplot(num_plots,1,5); hold on; grid on; box on;
        plot(time_resampled, -pos_z.Data);
        ylabel('Height [m]');
    end
    
    % compute the offset
    disp(['Mean of Pitch: ',num2str(mean(rad2deg(pitch.Data))),' m/s']);
    disp(['Mean of AoA: ',num2str(mean(rad2deg(aoa.Data))),' m/s']);
    disp(['AoA offset: ',num2str(mean(rad2deg(aoa.Data) - rad2deg(pitch.Data))),' deg']);
else
    disp('ERROR: logged topics are not sufficient for airspeed calibration.');
end
% ! END do not modify ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
