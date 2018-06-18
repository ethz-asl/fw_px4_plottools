function TelemRSSIPlots(sysvector, topics)
% Display telemetry and rc input connection data
% TODO: - Add the 3D rc loss position plot

fig1 = figure();
fig1.Name = 'Telemetry and RC Connection Data';

% compute general data
theta = timeseries(atan2d(sysvector('vehicle_local_position_0.y').Data, sysvector('vehicle_local_position_0.x').Data), ...
    sysvector('vehicle_local_position_0.x').Time);
dist_hor = timeseries(sqrt(sysvector('vehicle_local_position_0.x').Data.^2 +...
    sysvector('vehicle_local_position_0.y').Data.^2), sysvector('vehicle_local_position_0.x').Time);
dist_tot = timeseries(sqrt(sysvector('vehicle_local_position_0.x').Data.^2 +...
    sysvector('vehicle_local_position_0.y').Data.^2 + sysvector('vehicle_local_position_0.z').Data.^2),...
    sysvector('vehicle_local_position_0.x').Time);

% telemetry data
rssi_handle(1) = subplot(5,1,1);
hold on;
plot(sysvector('telemetry_status_0.rssi').Time, sysvector('telemetry_status_0.rssi').Data);
plot(sysvector('telemetry_status_0.remote_rssi').Time, sysvector('telemetry_status_0.remote_rssi').Data);
plot(sysvector('telemetry_status_0.noise').Time, sysvector('telemetry_status_0.noise').Data);
plot(sysvector('telemetry_status_0.remote_noise').Time, sysvector('telemetry_status_0.remote_noise').Data);
if topics.telemetry_status.num_instances > 1
    plot(sysvector('telemetry_status_1.rssi').Time, sysvector('telemetry_status_1.rssi').Data);
    plot(sysvector('telemetry_status_1.remote_rssi').Time, sysvector('telemetry_status_1.remote_rssi').Data);
    plot(sysvector('telemetry_status_1.noise').Time, sysvector('telemetry_status_1.noise').Data);
    plot(sysvector('telemetry_status_1.remote_noise').Time, sysvector('telemetry_status_1.remote_noise').Data);
    legend('TEL0 RSSI','TEL0 Remote RSSI', 'TEL0 Noise', 'TEL0 Remote Noise', 'TEL1 RSSI','TEL1 Remote RSSI', 'TEL1 Noise', 'TEL1 Remote Noise');
else
    legend('TEL0 RSSI','TEL0 Remote RSSI', 'TEL0 Noise', 'TEL0 Remote Noise');
end
hold off;
xlabel('Time [s]');

% rc data
rssi_handle(2) = subplot(5,1,2);
hold on;
plot(sysvector('input_rc_0.rssi').Time, sysvector('input_rc_0.rssi').Data);
max_rc_lost = max(sysvector('input_rc_0.rc_lost').Data);
if (max_rc_lost > 0.0)
    plot(sysvector('input_rc_0.rc_lost').Time, ...
        sysvector('input_rc_0.rc_lost').Data/max_rc_lost*max(sysvector('input_rc_0.rssi').Data));
else
    plot(sysvector('input_rc_0.rc_lost').Time, sysvector('input_rc_0.rc_lost').Data);
end
hold off;
legend('RC RSSI','RC Lost');
xlabel('Time [s]');

% telemetry package data
rssi_handle(3) = subplot(5,1,3);
hold on;
plot(sysvector('telemetry_status_0.fixed').Time, sysvector('telemetry_status_0.fixed').Data);
plot(sysvector('telemetry_status_0.rxerrors').Time, sysvector('telemetry_status_0.rxerrors').Data);
hold off;
legend('TEL0 Fixed RX packets','TEL0 Lost RX packets');
xlabel('Time [s]');

% attitude
rssi_handle(4) = subplot(5,1,4);
[pitch, roll, yaw] = ...
    QuaternionToEuler(sysvector('vehicle_attitude_0.q_0'), sysvector('vehicle_attitude_0.q_1'),...
    sysvector('vehicle_attitude_0.q_2'), sysvector('vehicle_attitude_0.q_3'));
hold on;
plot(roll.Time, roll.Data.*180./pi);
plot(pitch.Time, pitch.Data.*180./pi);
plot(yaw.Time, yaw.Data.*180./pi);
plot(theta.Time, theta.Data);
hold off;
legend({'roll','pitch','yaw','theta_home'}, 'Interpreter', 'none');
grid on;

% distances to home
rssi_handle(5) = subplot(5,1,5);
hold on;
plot(dist_hor.Time, dist_hor.Data);
plot(dist_tot.Time, dist_tot.Data);
plot(sysvector('vehicle_local_position_0.z').Time, -sysvector('vehicle_local_position_0.z').Data);
hold off;
legend('dist home hor [m]','dist home tot [m]','rel alt [m]');
grid on;

linkaxes(rssi_handle(:),'x');
set(rssi_handle(:),'XGrid','on','YGrid','on','ZGrid','on');
dcm_obj = datacursormode(fig1);
set(dcm_obj,'UpdateFcn',@HighPrecisionTooltipCallback);
end

