%%
clc;
close all;
clear;

%%
wind_n_mag = 5.0;
wind_e_mag = 2.0;
circle_radius = 100;
t_sim = 90;
dt_sim = 1.0;
nom_airspeed = 10;
aoa_offset_deg = -2;
beta_offset_deg = 4;
roll_angle = 0.0;

%% setup the raw data
times = 0:dt_sim:t_sim;
wind_n = wind_n_mag * ones(size(times));
wind_e = wind_e_mag * ones(size(times));
syms x;

beta_offset_rad = deg2rad(beta_offset_deg);

pos_n = zeros(size(times));
pos_e = zeros(size(times));
v_gnd_n = zeros(size(times));
v_gnd_e = zeros(size(times));
yaw = zeros(size(times));

%% simulate flying around
ground_distance = 0.0;
angle = ground_distance / circle_radius;

pos_n(1)= 0.0;
pos_e(1)= circle_radius;

tangent = [-sin(angle); cos(angle)];
wind_normal = wind_n(1) * tangent(1) - wind_e(1) * tangent(2);

solutions = solve(-wind_normal == nom_airspeed * sin(x) * tangent(1) - nom_airspeed * cos(x) * tangent(2), x, ...
        'Real', true);

for k = 1:length(solutions)
    airspeed_vector = [cos(solutions(k)) * nom_airspeed; sin(solutions(k)) * nom_airspeed];
    ve = airspeed_vector(1) + wind_e(1);
    vn = airspeed_vector(2) + wind_n(1);

    if (tangent(1) * ve + tangent(2) * vn) > 0.0
        v_gnd_e(1) = ve;
        v_gnd_n(1) = vn;
        yaw(1) = solutions(k);

    end
end


for i = 2:length(times)
    % position update based on travelled distance
    ground_distance = ground_distance + dt_sim * sqrt(v_gnd_n(i-1) * v_gnd_n(i-1) + v_gnd_e(i-1) * v_gnd_e(i-1));

    angle = ground_distance / circle_radius;

    pos_e(i) = cos(angle) * circle_radius;
    pos_n(i) = sin(angle) * circle_radius;

    % velocity + yaw update
    tangent = [-sin(angle); cos(angle)];
    wind_normal = wind_n(i) * tangent(1) - wind_e(i) * tangent(2);

    solutions = solve(-wind_normal == nom_airspeed * sin(x) * tangent(1) - nom_airspeed * cos(x) * tangent(2), x, ...
        'Real', true);

    for k = 1:length(solutions)
        airspeed_vector = [cos(solutions(k)) * nom_airspeed; sin(solutions(k)) * nom_airspeed];
        ve = airspeed_vector(1) + wind_e(i);
        vn = airspeed_vector(2) + wind_n(i);

        if (tangent(1) * ve + tangent(2) * vn) > 0.0
            v_gnd_e(i) = ve;
            v_gnd_n(i) = vn;
            yaw(i) = solutions(k);

        end
    end
end


%% plot the vectors
figure()
hold on;
plot(pos_e, pos_n)
quiver(pos_e, pos_n, nom_airspeed * cos(yaw), nom_airspeed * sin(yaw), 0, 'b')
quiver(pos_e + nom_airspeed * cos(yaw), pos_n + nom_airspeed * sin(yaw), wind_e, wind_n, 0, 'r')
quiver(pos_e, pos_n, v_gnd_e, v_gnd_n, 0, 'g')
hold off;
axis equal;


%% compute the wind with an error in the yaw
airspeed_vec_wrong = [cos(yaw + beta_offset_rad) * nom_airspeed; sin(yaw + beta_offset_rad) * nom_airspeed];

wind_est_e = v_gnd_e - airspeed_vec_wrong(1, :);
wind_est_n = v_gnd_n - airspeed_vec_wrong(2, :);

%% plotting of the results with the beta bias
figure()
subplot(2,1,1);
hold on;
plot(times, wind_e, '--b')
plot(times, wind_est_e, 'b')
hold off;
legend(['w_e true', 'w_e est'])

subplot(2,1,2);
hold on;
plot(times, wind_n, '--b')
plot(times, wind_est_n, 'b')
hold off;
legend(['w_n true', 'w_n est'])


figure()
hold on;
plot(pos_e, pos_n)
quiver(pos_e, pos_n, airspeed_vec_wrong(1, :), airspeed_vec_wrong(2, :), 0, 'b')
quiver(pos_e + airspeed_vec_wrong(1, :), pos_n + airspeed_vec_wrong(2, :), wind_est_e, wind_est_n, 0, 'r')
quiver(pos_e, pos_n, v_gnd_e, v_gnd_n, 0, 'g')
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
sysvector.vehicle_local_position_0.y = timeseries(pos_e', times');
sysvector.vehicle_local_position_0.z = timeseries(zeros(size(times')), times');

topics.vehicle_attitude.logged = true;
% hacky way of making the quaternion continuous...
yaw_tmp = wrapToPi(-(yaw' - 0.5 * pi));
q = angle2quat(yaw_tmp, 0.0 * yaw',  roll_angle * ones(size(times')));
factor = 1;
for i=2:length(q)
    if (abs(yaw_tmp(i-1) - yaw_tmp(i)) > 1.0)
        factor = factor * (-1);
    end
    q(i, :) = q(i, :) * factor;
end
sysvector.vehicle_attitude_0.q_0 = timeseries(squeeze(q(:,1)), times');
sysvector.vehicle_attitude_0.q_1 = timeseries(squeeze(q(:,2)), times');
sysvector.vehicle_attitude_0.q_2 = timeseries(squeeze(q(:,3)), times');
sysvector.vehicle_attitude_0.q_3 = timeseries(squeeze(q(:,4)), times');

topics.vehicle_gps_position.logged = true;
sysvector.vehicle_gps_position_0.vel_n_m_s = timeseries(v_gnd_n', times');
sysvector.vehicle_gps_position_0.vel_e_m_s = timeseries(v_gnd_e', times');
sysvector.vehicle_gps_position_0.vel_d_m_s = timeseries(zeros(size(times')), times');

% 0 gyro rates for now
topics.sensor_gyro.logged = true;
sysvector.sensor_gyro_0.x = timeseries(zeros(size(times')), times');
sysvector.sensor_gyro_0.y = timeseries(zeros(size(times')), times');
sysvector.sensor_gyro_0.z = timeseries(zeros(size(times')), times');

topics.airflow_aoa.logged = true;
sysvector.airflow_aoa_0.aoa_rad = timeseries(deg2rad(aoa_offset_deg) * ones(size(times')), times');

topics.airflow_slip.logged = true;
sysvector.airflow_slip_0.slip_rad = timeseries(deg2rad(beta_offset_deg) * ones(size(times')), times');
