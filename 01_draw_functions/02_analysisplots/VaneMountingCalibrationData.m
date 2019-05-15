function [mean_states, tspan1] = VaneMountingCalibrationData(sysvector, topics, paramvector, tspan)

% calibrated airflow angle measurements
aoa_meas = timeseries(deg2rad(1e-7 * paramvector.cal_hall_rev.Data(1) * ...
        (paramvector.cal_hall_p0.Data(1) + ...
        paramvector.cal_hall_p1.Data(1) .* sysvector.sensor_hall_0.mag_T.Data + ...
        paramvector.cal_hall_p2.Data(1) .* sysvector.sensor_hall_0.mag_T.Data .* sysvector.sensor_hall_0.mag_T.Data + ...
        paramvector.cal_hall_p3.Data(1) .* sysvector.sensor_hall_0.mag_T.Data .* sysvector.sensor_hall_0.mag_T.Data .* sysvector.sensor_hall_0.mag_T.Data)),...
        sysvector.sensor_hall_0.mag_T.Time);
slip_meas = timeseries(deg2rad(1e-7 * paramvector.cal_hall_01_rev.Data(1) * ...
        (paramvector.cal_hall_01_p0.Data(1) + ...
        paramvector.cal_hall_01_p1.Data(1) .* sysvector.sensor_hall_01_0.mag_T.Data + ...
        paramvector.cal_hall_01_p2.Data(1) .* sysvector.sensor_hall_01_0.mag_T.Data .* sysvector.sensor_hall_01_0.mag_T.Data + ...
        paramvector.cal_hall_01_p3.Data(1) .* sysvector.sensor_hall_01_0.mag_T.Data .* sysvector.sensor_hall_01_0.mag_T.Data .* sysvector.sensor_hall_01_0.mag_T.Data)),...
        sysvector.sensor_hall_01_0.mag_T.Time);

% synchronise the data
dt = 0.05;
min_time = max([tspan(1), ...
    sysvector.vehicle_gps_position_0.vel_n_m_s.Time(1),...
    sysvector.airspeed_0.true_airspeed_m_s.Time(1),...
    sysvector.vehicle_attitude_0.q_0.Time(1),...
    sysvector.sensor_hall_0.mag_T.Time(1),...
    sysvector.sensor_hall_01_0.mag_T.Time(1)]);
max_time = min([tspan(2), ...
    sysvector.vehicle_gps_position_0.vel_n_m_s.Time(end),...
    sysvector.airspeed_0.true_airspeed_m_s.Time(end),...
    sysvector.vehicle_attitude_0.q_0.Time(end),...
    sysvector.sensor_hall_0.mag_T.Time(end),...
    sysvector.sensor_hall_01_0.mag_T.Time(end)]);
if (topics.vehicle_local_position.logged)
    min_time = max([min_time, sysvector.vehicle_local_position_0.x.Time(1)]);
    max_time = min([max_time, sysvector.vehicle_local_position_0.x.Time(end)]);
end
time_resampled = min_time:dt:max_time;
tspan1 = [min_time, max_time];
len_t = length(time_resampled);

% resample inputs / outputs
aoa_meas = resample(aoa_meas, time_resampled);
slip_meas = resample(slip_meas, time_resampled);
vel_n = resample(sysvector.vehicle_gps_position_0.vel_n_m_s, time_resampled);
vel_e = resample(sysvector.vehicle_gps_position_0.vel_e_m_s, time_resampled);
vel_d = resample(sysvector.vehicle_gps_position_0.vel_d_m_s, time_resampled);
airspeed = resample(sysvector.airspeed_0.true_airspeed_m_s, time_resampled);
q_0 = resample(sysvector.vehicle_attitude_0.q_0, time_resampled);
q_1 = resample(sysvector.vehicle_attitude_0.q_1, time_resampled);
q_2 = resample(sysvector.vehicle_attitude_0.q_2, time_resampled);
q_3 = resample(sysvector.vehicle_attitude_0.q_3, time_resampled);

% reconstruct airspeed vector in inertial frame
Hi2b = quat2dcm([q_0.Data, q_1.Data, q_2.Data, q_3.Data]);
vel_airsp = zeros(len_t, 3);
for i = 1:len_t
    vel_airsp(i,:) = (Hi2b(:,:,i)' * airspeed.Data(i) * [1; tan(slip_meas.Data(i)); tan(aoa_meas.Data(i))])';
end

% reconstruct wind velocity
vel_wind = [vel_n.Data, vel_e.Data, vel_d.Data] - vel_airsp;

% calculate mean wind components
wind_n_mean = mean(vel_wind(:,1));
wind_e_mean = mean(vel_wind(:,2));
slip_mean = mean(slip_meas.Data);
mean_states = [wind_n_mean; wind_e_mean; rad2deg(slip_mean)];

% / position plot / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
if (topics.vehicle_local_position.logged)
    figure('color','w','name','Position with Wind Vector');

    pos_x = resample(sysvector.vehicle_local_position_0.x, time_resampled);
    pos_y = resample(sysvector.vehicle_local_position_0.y, time_resampled);
    pos_z = resample(sysvector.vehicle_local_position_0.z, time_resampled);
    
    % make z up
    pos_z.Data = -pos_z.Data;
    vel_d.Data = -vel_d.Data;

    plot3(pos_x.Data,pos_y.Data,pos_z.Data,'k.');
    axis equal
    hold all
    text(pos_x.Data(1),pos_y.Data(1),pos_z.Data(1),'t_{min}');
    text(pos_x.Data(end),pos_y.Data(end),pos_z.Data(end),'t_{max}');
    di = numel(pos_x.Data);
    distep = floor(di/20);
%     for i=1:20
%         text(pos_x.Data(i*distep), ...
%             pos_y.Data(i*distep), ...
%             pos_z.Data(i*distep), ...
%             num2str(pos_x.Time(i*distep,1), '%.1f'));
%     end

    quiver3(pos_x.Data, pos_y.Data, pos_z.Data,...
            vel_wind(:,1), vel_wind(:,2), zeros(len_t,1), 0);
    legend('pos','v_{wind}');
    title('Wind Data');
    xlabel('delta-Longitude [m]');
    ylabel('delta-Latitude [m]');
    zlabel('Altitude above MSL [m]');
    grid on
end

% / wind plot / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
figure('color','w','name','Reconstructed Airflow Angles and Wind');

lines_ = lines(7);
plot_opacity = 0.5;

wind(1) = subplot(4,1,1); hold on; grid on; box on;
plot(time_resampled, rad2deg(aoa_meas.Data), 'color', lines_(1,:));
plot(time_resampled, rad2deg(slip_meas.Data), 'color', lines_(2,:));
legend('\alpha meas.', '\beta meas.');
ylabel('Airflow Angles [deg]');

wind(2) = subplot(4,1,2); hold on; grid on; box on;
plot(time_resampled, vel_wind(:,1), '-.', 'color', lines_(1,:) * plot_opacity + ones(1,3) * (1 - plot_opacity));
plot(time_resampled, vel_wind(:,2), '-.', 'color', lines_(2,:) * plot_opacity + ones(1,3) * (1 - plot_opacity));
plot(time_resampled, vel_wind(:,3), '-.', 'color', lines_(3,:) * plot_opacity + ones(1,3) * (1 - plot_opacity));
plot(time_resampled([1 end]), wind_n_mean * ones(1,2), 'linewidth', 2, 'color', lines_(1,:));
plot(time_resampled([1 end]), wind_e_mean * ones(1,2), 'linewidth', 2, 'color', lines_(2,:));
plot(time_resampled([1 end]), zeros(1,2), 'linewidth', 2, 'color', lines_(3,:));
legend('w_n recnstr.', 'w_e recnstr.', 'w_d recnstr.', 'w_n mean', 'w_e mean', 'w_d = ZERO');
ylabel('Wind Components [m/s]');

wind(3) = subplot(4,1,3); hold on; grid on; box on;
plot(time_resampled, sqrt(vel_n.Data.^2 + vel_e.Data.^2));
plot(time_resampled, airspeed.Data);
plot(time_resampled, sqrt(vel_wind(:,1).^2+vel_wind(:,2).^2));
ylabel('Speeds [m/s]');
legend('2D gnd. sp.', 'u (x-body) airsp.', '2D wind sp.');

wind(4) = subplot(4,1,4); hold on; grid on; box on;
if (topics.vehicle_local_position.logged)
    plot(time_resampled, pos_z.Data);
    ylabel('Height [m]');
else
    text((time_resampled(end)-time_resampled(1))/2+time_resampled(1), 0, 'no position data logged', ...
        'fontsize', 14, 'horizontalalignment', 'center');
    ylim([-1, 1]);
end

xlabel('Time [s]');
linkaxes(wind(:),'x');
xlim(wind(:), time_resampled([1 end]));

end

