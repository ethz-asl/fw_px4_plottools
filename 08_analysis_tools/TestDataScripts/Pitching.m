%%
clc;
close all;
clear;

%%
wind_d_mag = -1.0;
pitch_period = 30;
max_pitch_deg = 30;
t_sim = 90;
dt_sim = 1.0;
nom_airspeed = 10;
aoa_offset_deg = -5;

%% setup the raw data
times = 0:dt_sim:t_sim;
wind_d = wind_d_mag * ones(size(times));
wind_n = 0.0 * ones(size(times));
syms x;

aoa_offset_rad = deg2rad(aoa_offset_deg);
max_pitch_rad = deg2rad(max_pitch_deg);

pos_n = zeros(size(times));
pos_d = zeros(size(times));
v_gnd_n = zeros(size(times));
v_gnd_d = zeros(size(times));
pitch = zeros(size(times));

%% simulate flying around
for k = 1:length(times)
    pitch(k) = sin(2*pi*k*dt_sim/pitch_period) * max_pitch_rad;
    v_gnd_n(k) = cos(-pitch(k)) * nom_airspeed;
    v_gnd_d(k) = sin(-pitch(k)) * nom_airspeed + wind_d(k);
    if (k>1)
        pos_n(k) = pos_n(k-1) + dt_sim * v_gnd_n(k-1);
        pos_d(k) = pos_d(k-1) + dt_sim * v_gnd_d(k-1);
    end
end

%% plot the vectors
figure()
hold on;
plot(pos_n, -pos_d)
quiver(pos_n, -pos_d, nom_airspeed * cos(pitch), nom_airspeed * sin(pitch), 0, 'b')
quiver(pos_n + nom_airspeed * cos(pitch), -pos_d + nom_airspeed * sin(pitch), zeros(size(wind_d)), -wind_d, 0, 'r')
quiver(pos_n, -pos_d, v_gnd_n, -v_gnd_d, 0, 'g')
hold off;
axis equal;


%% compute the wind with an error in the yaw
airspeed_vec_wrong = [cos(-pitch + aoa_offset_rad) * nom_airspeed; sin(-pitch + aoa_offset_rad) * nom_airspeed];

wind_est_n = v_gnd_n - airspeed_vec_wrong(1, :);
wind_est_d = v_gnd_d - airspeed_vec_wrong(2, :);

%% plotting of the results with the beta bias
figure()
subplot(2,1,1);
hold on;
plot(times, wind_n, '--b')
plot(times, wind_est_n, 'b')
hold off;
legend(['w_n true', 'w_n est'])

subplot(2,1,2);
hold on;
plot(times, wind_d, '--b')
plot(times, wind_est_d, 'b')
hold off;
legend(['w_d true', 'w_d est'])


figure()
hold on;
plot(pos_n, -pos_d)
quiver(pos_n, -pos_d, airspeed_vec_wrong(1, :), -airspeed_vec_wrong(2, :), 0, 'b')
quiver(pos_n + airspeed_vec_wrong(1, :), -pos_d - airspeed_vec_wrong(2, :), wind_est_n, -wind_est_d, 0, 'r')
quiver(pos_n, -pos_d, v_gnd_n, -v_gnd_d, 0, 'g')
hold off;
axis equal;

%% generating a fake topics and sysvector
topics.differential_pressure.logged = true;
sysvector.differential_pressure_0.differential_pressure_raw_pa = ...
    timeseries(squeeze(1.225 * 0.5 * nom_airspeed * nom_airspeed * ones(size(times'))), times);
sysvector.differential_pressure_0.temperature = ...
    timeseries(20.0 * ones(size(times')), times');

topics.sensor_baro.logged = true;
sysvector.sensor_baro_0.pressure = timeseries(900.0 * ones(size(times')), times');

topics.airspeed.logged = true;
sysvector.airspeed_0.true_airspeed_m_s = timeseries(nom_airspeed * ones(size(times')), times');

topics.vehicle_local_position.logged = true;
sysvector.vehicle_local_position_0.x = timeseries(pos_n', times');
sysvector.vehicle_local_position_0.y = timeseries(zeros(size(times')), times');
sysvector.vehicle_local_position_0.z = timeseries(pos_d', times');

topics.vehicle_attitude.logged = true;
q = angle2quat(0.0 * ones(size(times')), 0.0 * pitch',  0.0 * ones(size(times')));
sysvector.vehicle_attitude_0.q_0 = timeseries(squeeze(q(:,1)), times');
sysvector.vehicle_attitude_0.q_1 = timeseries(squeeze(q(:,2)), times');
sysvector.vehicle_attitude_0.q_2 = timeseries(squeeze(q(:,3)), times');
sysvector.vehicle_attitude_0.q_3 = timeseries(squeeze(q(:,4)), times');

topics.vehicle_gps_position.logged = true;
sysvector.vehicle_gps_position_0.vel_n_m_s = timeseries(v_gnd_n', times');
sysvector.vehicle_gps_position_0.vel_e_m_s = timeseries(zeros(size(times')), times');
sysvector.vehicle_gps_position_0.vel_d_m_s = timeseries(v_gnd_d', times');

% 0 gyro rates for now
topics.sensor_gyro.logged = true;
sysvector.sensor_gyro_0.x = timeseries(zeros(size(times')), times');
sysvector.sensor_gyro_0.y = timeseries(zeros(size(times')), times');
sysvector.sensor_gyro_0.z = timeseries(zeros(size(times')), times');

topics.airflow_aoa.logged = true;
sysvector.airflow_aoa_0.aoa_rad = timeseries(deg2rad(aoa_offset_deg) * ones(size(times')), times');

topics.airflow_slip.logged = true;
sysvector.airflow_slip_0.slip_rad = timeseries(zeros(size(times')), times');
