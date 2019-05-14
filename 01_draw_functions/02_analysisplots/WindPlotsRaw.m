function [mean_raw_wind, tspan1] = WindPlotsRaw(sysvector, topics, tspan)
% Display the wind data.

figure('color', 'w', 'name', 'Reconstructed Wind 3D');
% synchronise the data
dt = 0.05;
min_time = max([tspan(1), ...
    sysvector.vehicle_gps_position_0.vel_n_m_s.Time(1),...
    sysvector.airspeed_0.true_airspeed_m_s.Time(1),...
    sysvector.vehicle_attitude_0.q_0.Time(1)]);
max_time = min([tspan(2), ...
    sysvector.vehicle_gps_position_0.vel_n_m_s.Time(end),...
    sysvector.airspeed_0.true_airspeed_m_s.Time(end),...
    sysvector.vehicle_attitude_0.q_0.Time(end)]);
if (topics.vehicle_local_position.logged)
    min_time = max([min_time, sysvector.vehicle_local_position_0.x.Time(1)]);
    max_time = min([max_time, sysvector.vehicle_local_position_0.x.Time(end)]);
end
time_resampled = min_time:dt:max_time;
tspan1 = [min_time, max_time];

vel_n = resample(sysvector.vehicle_gps_position_0.vel_n_m_s, time_resampled);
vel_e = resample(sysvector.vehicle_gps_position_0.vel_e_m_s, time_resampled);
vel_d = resample(sysvector.vehicle_gps_position_0.vel_d_m_s, time_resampled);
airspeed = resample(sysvector.airspeed_0.true_airspeed_m_s, time_resampled);
q_0 = resample(sysvector.vehicle_attitude_0.q_0, time_resampled);
q_1 = resample(sysvector.vehicle_attitude_0.q_1, time_resampled);
q_2 = resample(sysvector.vehicle_attitude_0.q_2, time_resampled);
q_3 = resample(sysvector.vehicle_attitude_0.q_3, time_resampled);
[yaw, ~, ~] = quat2angle([q_0.Data, q_1.Data, q_2.Data, q_3.Data]);

% reconstructed horizontal wind with zero sideslip and zero vertical motion
% assumptions
wind_e = vel_e.Data - airspeed.Data .* sin(yaw);
wind_n = vel_n.Data - airspeed.Data .* cos(yaw);

% calculate mean wind components
wind_e_mean = mean(wind_e);
wind_n_mean = mean(wind_n);
mean_raw_wind = [wind_e_mean, wind_n_mean];

% / position plot / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
if (topics.vehicle_local_position.logged)
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
    for i=1:20
        text(pos_x.Data(i*distep), ...
            pos_y.Data(i*distep), ...
            pos_z.Data(i*distep), ...
            num2str(pos_x.Time(i*distep,1), '%.1f'));
    end

    quiver3(pos_x.Data, pos_y.Data, pos_z.Data,...
            wind_n, wind_e, zeros(size(wind_n)), 0);
    legend('pos','v_{wind}');
    title('Wind Data');
    xlabel('delta-Longitude [m]');
    ylabel('delta-Latitude [m]');
    zlabel('Altitude above MSL [m]');
    grid on
end

% / wind plot / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
figure('color','w','name','Reconstructed Wind Plot');

wind(1) = subplot(3,1,1); hold on; grid on; box on;
plot(time_resampled, wind_e);
plot(time_resampled, wind_n);
plot(time_resampled([1 end]), wind_e_mean * ones(1,2), 'linewidth', 2);
plot(time_resampled([1 end]), wind_n_mean * ones(1,2), 'linewidth', 2);
legend('wind east', 'wind north', 'wind east mean', 'wind north mean');
ylabel('Wind Components [m/s]');

wind(2) = subplot(3,1,2); hold on; grid on; box on;
plot(time_resampled, sqrt(vel_n.Data.^2 + vel_e.Data.^2));
plot(time_resampled, airspeed.Data);
plot(time_resampled, sqrt(wind_n.^2 + wind_e.^2));
ylabel('Speeds [m/s]');
legend('ground sp.', 'airspeed', 'wind sp.');

wind(3) = subplot(3,1,3); hold on; grid on; box on;
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

