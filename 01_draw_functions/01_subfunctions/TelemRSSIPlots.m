function TelemRSSIPlots(sysvector, topics)
% Display telemetry and rc input connection data
% TODO: - Add the 3D rc loss position plot


fig1 = figure();
fig1.Name = 'Telemetry and RC Connection Data';

% compute general data
if topics.vehicle_local_position.logged
    theta = timeseries(atan2d(sysvector.vehicle_local_position_0.y.Data, sysvector.vehicle_local_position_0.x.Data), ...
        sysvector.vehicle_local_position_0.x.Time);
    dist_hor = timeseries(sqrt(sysvector.vehicle_local_position_0.x.Data.^2 +...
        sysvector.vehicle_local_position_0.y.Data.^2), sysvector.vehicle_local_position_0.x.Time);
    dist_tot = timeseries(sqrt(sysvector.vehicle_local_position_0.x.Data.^2 +...
        sysvector.vehicle_local_position_0.y.Data.^2 + sysvector.vehicle_local_position_0.z.Data.^2),...
        sysvector.vehicle_local_position_0.x.Time);
end

telem_data_available = false;
if topics.radio_status.logged
    telem_data = [getfield(sysvector, 'radio_status_' + string(0))];
    for i = 1:topics.radio_status.num_instances-1
        telem_data = getfield(sysvector, 'radio_status_' + string(i));
    end
    telem_data_available = true;
    num_instances = topics.radio_status.num_instances;
elseif topics.telemetry_status.logged
    if isfield(sysvector.telemetry_status_0, 'rssi')
        telem_data = [getfield(sysvector, 'telemetry_status_' + string(0))];
        for i = 1:topics.telemetry_status.num_instances-1
            telem_data = getfield(sysvector, 'telemetry_status_' + string(i));
        end
        telem_data_available = true;
        num_instances = topics.telemetry_status.num_instances;
    end
end

% telemetry data
rssi_handle(1) = subplot(5,1,1);
if telem_data_available
    hold on;
    legend_entries = {};
    for i = 0:num_instances-1
        plot(telem_data(i+1).rssi.Time, telem_data(i+1).rssi.Data);
        plot(telem_data(i+1).remote_rssi.Time, telem_data(i+1).remote_rssi.Data);
        plot(telem_data(i+1).noise.Time, telem_data(i+1).noise.Data);
        plot(telem_data(i+1).remote_noise.Time, telem_data(i+1).remote_noise.Data);
        legend_entries = [legend_entries; {'TEL' + string(i) + ' RSSI'; 'TEL' + string(i) + ' Remote RSSI'; 'TEL' + string(i) + ' Noise'; 'TEL' + string(i) + ' Remote Noise'}];
    end

    legend(legend_entries);
    hold off;
end

% rc data
rssi_handle(2) = subplot(5,1,2);
if topics.input_rc.logged
    hold on;
    plot(sysvector.input_rc_0.rssi.Time, sysvector.input_rc_0.rssi.Data);
    max_rc_lost = max(sysvector.input_rc_0.rc_lost.Data);
    if (max_rc_lost > 0.0)
        plot(sysvector.input_rc_0.rc_lost.Time, ...
            sysvector.input_rc_0.rc_lost.Data/max_rc_lost*max(sysvector.input_rc_0.rssi.Data));
    else
        plot(sysvector.input_rc_0.rc_lost.Time, sysvector.input_rc_0.rc_lost.Data);
    end
    hold off;
    legend('RC RSSI','RC Lost');
    xlabel('Time [s]');
end

% telemetry package data
rssi_handle(3) = subplot(5,1,3);
if telem_data_available
   hold on;
   legend_entries = {};
    for i = 0:num_instances-1
        plot(telem_data(i+1).fixed.Time, telem_data(i+1).fixed.Data);
        plot(telem_data(i+1).rxerrors.Time, telem_data(i+1).rxerrors.Data);
        legend_entries = [legend_entries; {'TEL' + string(i) + ' Fixed RX packets'; 'TEL' + string(i) + ' Lost RX packets'}];
    end

    legend(legend_entries);
    hold off;
end

% attitude
rssi_handle(4) = subplot(5,1,4);
legend_entries = {};
hold on;
if topics.vehicle_attitude.logged
    [pitch, roll, yaw] = ...
        QuaternionToEuler(sysvector.vehicle_attitude_0.q_0, sysvector.vehicle_attitude_0.q_1,...
        sysvector.vehicle_attitude_0.q_2, sysvector.vehicle_attitude_0.q_3);
    plot(roll.Time, roll.Data.*180./pi);
    plot(pitch.Time, pitch.Data.*180./pi);
    plot(yaw.Time, yaw.Data.*180./pi);
    legend_entries = [legend_entries; {'roll';'pitch';'yaw'}];
end
if topics.vehicle_local_position.logged
    plot(theta.Time, theta.Data);
    legend_entries = [legend_entries; {'theta_home'}];

end
legend(legend_entries, 'Interpreter', 'none');
hold off;
grid on;

% distances to home
rssi_handle(5) = subplot(5,1,5);
if topics.vehicle_local_position.logged
    hold on;
    plot(dist_hor.Time, dist_hor.Data);
    plot(dist_tot.Time, dist_tot.Data);
    plot(sysvector.vehicle_local_position_0.z.Time, -sysvector.vehicle_local_position_0.z.Data);
    hold off;
    legend('dist home hor [m]','dist home tot [m]','rel alt [m]');
    grid on;
    xlabel('Time [s]');
end

linkaxes(rssi_handle(:),'x');
set(rssi_handle(:),'XGrid','on','YGrid','on','ZGrid','on');
dcm_obj = datacursormode(fig1);
set(dcm_obj,'UpdateFcn',@HighPrecisionTooltipCallback);
end

