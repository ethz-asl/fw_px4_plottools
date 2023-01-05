% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
% Airflow Calibration solver / / / / / / / / / / / / / / / / / / / / / / / / / /
function [xopt, opt_info, mean_gsp_err, std_gsp_err, mean_wind_d, optimization_data] = SolveAirflowCalibrationDynamic(sysvector, topics, paramvector, params, x0, lb, ub, config)

% TODO: 
%  - Divide data into segments with the constant wind assumption

input = struct();
time_resampled = [];
window_lengths = [];
dt_rs = 0.05;

[aoa, slip, aoa_logged, slip_logged] = CalibrateAirflowAngles(sysvector, topics, paramvector, params, config);
if (~aoa_logged || ~slip_logged)
    error('Slip and AoA measurements are required')
end

for i = 1:length(config.t_ends)
    time = config.t_starts(i):dt_rs:config.t_ends(i);
    window_lengths = [window_lengths length(time)];
    time_resampled = [time_resampled time]; 
end

acc_x = resample(sysvector.sensor_accel_0.x, time_resampled(1):dt_rs:time_resampled(end));
acc_y = resample(sysvector.sensor_accel_0.y, time_resampled(1):dt_rs:time_resampled(end));
acc_z = resample(sysvector.sensor_accel_0.z, time_resampled(1):dt_rs:time_resampled(end));
gyr_x = resample(sysvector.sensor_gyro_0.x, time_resampled(1):dt_rs:time_resampled(end));
gyr_y = resample(sysvector.sensor_gyro_0.y, time_resampled(1):dt_rs:time_resampled(end));
gyr_z = resample(sysvector.sensor_gyro_0.z, time_resampled(1):dt_rs:time_resampled(end));

if config.t_movmean > 0
    acc_x.Data = movmean(acc_x.Data, ceil(config.t_movmean / dt_rs));
    acc_y.Data = movmean(acc_y.Data, ceil(config.t_movmean / dt_rs));
    acc_z.Data = movmean(acc_z.Data, ceil(config.t_movmean / dt_rs));
    gyr_x.Data = movmean(gyr_x.Data, ceil(config.t_movmean / dt_rs));
    gyr_y.Data = movmean(gyr_y.Data, ceil(config.t_movmean / dt_rs));
    gyr_z.Data = movmean(gyr_z.Data, ceil(config.t_movmean / dt_rs));
end

input.window_lengths = window_lengths;
input.dp = ResampleData(sysvector.differential_pressure_0.differential_pressure_raw_pa, time_resampled);
input.baro = ResampleData(sysvector.sensor_baro_0.pressure * 100, time_resampled);
input.temp = ResampleData(sysvector.differential_pressure_0.temperature, time_resampled);
q_0 = resample(sysvector.vehicle_attitude_0.q_0, time_resampled);
q_1 = resample(sysvector.vehicle_attitude_0.q_1, time_resampled);
q_2 = resample(sysvector.vehicle_attitude_0.q_2, time_resampled);
q_3 = resample(sysvector.vehicle_attitude_0.q_3, time_resampled);
input.rotm = quat2rotm([q_0.Data, q_1.Data, q_2.Data, q_3.Data]);
[pitch, roll, yaw] = QuaternionToEuler(q_0, q_1, q_2, q_3);
input.rotm = eul2rotm([yaw.Data + deg2rad(config.magnetic_declination_deg), pitch.Data, roll.Data], 'ZYX');
input.pitch = pitch.data;
input.roll = roll.data;
input.yaw = yaw.data + deg2rad(config.magnetic_declination_deg);
input.gspn = ResampleData(sysvector.vehicle_local_position_0.vx, time_resampled);
input.gspe = ResampleData(sysvector.vehicle_local_position_0.vy, time_resampled);
input.gspd = ResampleData(sysvector.vehicle_local_position_0.vz, time_resampled);
input.gyro_x = ResampleData(gyr_x, time_resampled);
input.gyro_y = ResampleData(gyr_y, time_resampled);
input.gyro_z = ResampleData(gyr_z, time_resampled);
input.acc_x = ResampleData(acc_x, time_resampled);
input.acc_y = ResampleData(acc_y, time_resampled);
input.acc_z = ResampleData(acc_z, time_resampled);

acc = [input.acc_x'; input.acc_y'; input.acc_z'];
acc_z_I = sum(squeeze(input.rotm(3,:,:)) .* acc, 1)';

if topics.actuator_controls_0.logged
    input.throttle = ResampleData(sysvector.actuator_controls_0_0.control_3, time_resampled);
end

input.aoa = ResampleData(aoa, time_resampled);
input.slip = ResampleData(slip, time_resampled);

if config.verbose
    options = optimoptions(@lsqnonlin,'Display','final');
else
    options = optimoptions(@lsqnonlin,'Display','off');
end

% quality measure
if config.use_accz_weighting
    input.weights = normpdf(abs(acc_z_I / 9.81 + 1), 0, config.weighting_sigma) / normpdf(0, 0, config.weighting_sigma);
else
    input.weights = ones(size(time_resampled'));
end

[~,out_before] = Dp2SpAnglesDynamic(x0,input,config);

if config.global_search
    ms = MultiStart('PlotFcns',@gsplotbestf, 'UseParallel', true, 'StartPointsToRun','bounds-ineqs');
    problem = createOptimProblem('lsqnonlin','x0',x0,'objective',@(x)Dp2SpAnglesDynamic(x,input,config),...
        'lb',lb,'ub',ub', 'options', options);
    [xopt,resnorm] = run(ms,problem,config.global_search_num_points);
    residual = 0;
    exitflag = 0;
    output = 0;
else
    [xopt,resnorm,residual,exitflag,output] = ...
        lsqnonlin(@(x)Dp2SpAnglesDynamic(x,input,config), x0, lb, ub, options); 
end

opt_info.resnorm = resnorm;
opt_info.residual = residual;
opt_info.exitflag = exitflag;
opt_info.output = output;

% calculate outputs
[~,out] = Dp2SpAnglesDynamic(xopt,input,config);

% mean / std of ground speed errors
mean_gsp_err = mean([input.gspn; input.gspe] - [out.gspn; out.gspe]);
std_gsp_err = std([input.gspn; input.gspe] - [out.gspn; out.gspe]);
mean_wind_d = mean(out.wind_d);

%% Get loiter characteristics
optimization_data.time = time_resampled;
optimization_data.roll = input.roll;
optimization_data.airspeed_total = sqrt(out.va_n.^2+out.va_e.^2+out.va_d.^2);
optimization_data.airspeed = out.airspeed_true;
optimization_data.g_load = input.acc_z/9.81;
if topics.actuator_controls_0.logged
    optimization_data.throttle = input.throttle;
else
    optimization_data.throttle = 0.0;
end

scale_factor = xopt(1);
if config.airspeed_scale_dynamic
    start_idx_bias = 4;
else
    start_idx_bias = 2;
end
input.airspeed = out.airspeed_true;
[aoa_bias, slip_bias, ~] = getAirflowAngleBias(xopt, config, input, start_idx_bias);

optimization_data.slip = input.slip - slip_bias;
optimization_data.uncorrected_slip = input.slip;
optimization_data.aoa = input.aoa - aoa_bias;
optimization_data.uncorrected_aoa = input.aoa;
optimization_data.aoa_bias = aoa_bias;
optimization_data.slip_bias = slip_bias;
optimization_data.scale_factor = scale_factor;
optimization_data.wn = out.wind_n;
optimization_data.we = out.wind_e;
optimization_data.wn_fit = out.wn;
optimization_data.we_fit = out.we;
optimization_data.wd = out.wind_d;
optimization_data.wn_raw = out_before.wind_n;
optimization_data.we_raw = out_before.wind_e;
optimization_data.wd_raw = out_before.wind_d;

optimization_data.gspn = input.gspn;
optimization_data.gspe = input.gspe;
optimization_data.gspd = input.gspd;
optimization_data.va_n = out.va_n;
optimization_data.va_e = out.va_e;
optimization_data.va_d = out.va_d;

%% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / 
% Output function / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
lines_ = lines(7);
plot_opacity = 0.5;
time_resampled = time_resampled / 60;

figure('color','w');

result_plots(1) = subplot(2,1,1); hold on; grid on; box on;
plot(time_resampled, out_before.wind_n);
plot(time_resampled, out_before.wind_e);
plot(time_resampled, out_before.wind_d);
plot(time_resampled, out.wn, 'linewidth', 2);
plot(time_resampled, out.we, 'linewidth', 2);
plot(time_resampled, out.wind_d, 'linewidth', 2);
legend('w_n raw','w_e raw', 'w_d raw','w_n fit','w_e fit', 'w_d fit');
ylabel('wind sp. [m/s]');

result_plots(2) = subplot(2,1,2); hold on; grid on; box on;
plot(time_resampled, sqrt(input.gspn.^2 + input.gspe.^2), '-.', 'color', lines_(1,:) * plot_opacity + ones(1,3) * (1 - plot_opacity));
plot(time_resampled, sqrt(out.gspn.^2 + out.gspe.^2), 'color', lines_(1,:));
plot(time_resampled, sqrt(out_before.wind_n.^2 + out_before.wind_e.^2), '-.', 'color', lines_(2,:) * plot_opacity + ones(1,3) * (1 - plot_opacity));
plot(time_resampled, sqrt(out.wn.^2 + out.we.^2), 'color', lines_(2,:));
plot(time_resampled, out_before.airspeed_true, '-.', 'color', lines_(3,:) * plot_opacity + ones(1,3) * (1 - plot_opacity));
plot(time_resampled, out.airspeed_true, 'color', lines_(3,:));

legend('gsp. data','gsp. fit','wind data','wind fit','airsp. data','airsp. cal');
ylabel('speeds [m/s]');
xlabel('Time [s]');
linkaxes(result_plots(:),'x');
xlim(result_plots(:), time_resampled([1 end]));


figure('color','w');

tmp_plots(1) = subplot(3,1,1); hold on; grid on; box on;
plot(time_resampled, out_before.wind_n);
plot(time_resampled, out_before.wind_e);
plot(time_resampled, out_before.wind_d);
legend('w_n','w_e', 'w_d');
ylabel('Wind Speed [m/s]');

tmp_plots(2) = subplot(3,1,2); hold on; grid on; box on;
plot(time_resampled, out.wn);
plot(time_resampled, out.we);
plot(time_resampled, out.wind_d);
legend('w_n','w_e', 'w_d');
ylabel('Wind Speed [m/s]');

tmp_plots(3) = subplot(3,1,3); hold on; grid on; box on;
plot(time_resampled, out.wind_n);
plot(time_resampled, out.wind_e);
plot(time_resampled, out.wind_d);
legend('w_n','w_e', 'w_d');
ylabel('Wind Speed [m/s]');

xlabel('Time [s]');
linkaxes(tmp_plots(:),'x');
xlim(tmp_plots(:), time_resampled([1 end]));

figure('color','w');
validation_plots(1) = subplot(2,1,1); hold on; grid on; box on;
plot(time_resampled, out.wind_n, 'color', lines_(1,:));
plot(time_resampled, movmean(out.wind_n, 200), 'color', lines_(2,:));
plot(time_resampled, out.wind_e, 'color', lines_(3,:));
plot(time_resampled, movmean(out.wind_e, 200), 'color', lines_(4,:));
plot(time_resampled, input.yaw, 'color', lines_(5,:));
legend('w_n opt', 'w_n opt filtered', 'w_e opt', 'w_e opt filtered', 'yaw');
ylabel('speeds | angles [m/s | deg]');

validation_plots(2) = subplot(2,1,2); hold on; grid on; box on;
plot(time_resampled, out.wind_d, 'color', lines_(1,:));
plot(time_resampled, movmean(out.wind_d, 200), 'color', lines_(2,:));
plot(time_resampled, input.gspd, 'color', lines_(3,:));
plot(time_resampled, rad2deg(input.pitch), 'color', lines_(4,:));
ylabel('[m/s | deg | m]');
legend('w_d opt', 'w_d opt filtered', 'groundspeed down', 'pitch');


xlabel('Time [s]');
linkaxes(validation_plots(:),'x');
xlim(validation_plots(:), time_resampled([1 end]));


function data = ResampleData(timeseries, time)
    ret = resample(timeseries, time, 'linear');
    data = ret.data;
end

function wind = GenerateWind(wind_windows, window_lengths)
    wind = ones(sum(window_lengths), 1);
    start_idx = 1;
    for idx = 1:length(wind_windows)
        end_idx = start_idx+window_lengths(idx)-1;
        wind(start_idx:end_idx) = wind_windows(idx);
        
        start_idx = end_idx+1;
    end
end
% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / 
% Output function / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
function [f, out] = Dp2SpAnglesDynamic(x,input, config)

out = struct();

% current solution
num_windows = length(config.t_ends);
config.airspeed_scale_factor = x(1);
if config.airspeed_scale_dynamic
    config.sfPaspd = x(2);
    config.sfPgyrz = x(3);
    start_idx = 4;
else
    start_idx = 2;
end

% calculate the airspeed from the raw dp measurements
[out.airspeed_true, ~] = CalculateAirspeed(input.dp, input.baro, input.temp, input.gyro_z, config);

% calculate the airflow angle biases
input.airspeed = out.airspeed_true;
[b_aoa, b_slip, start_idx_wind] = getAirflowAngleBias(x, config, input, start_idx);

% optionally correct for the measurement of the pitot tube
if config.airspeed_angle_correction
    airflow_angle_dist = rad2deg(sqrt((input.slip - b_slip).^2 + (input.aoa - b_aoa).^2));
    dv = (0.15 * airflow_angle_dist - 0.0044 * airflow_angle_dist .^2);
    out.airspeed_true = out.airspeed_true - dv;
end

% extract the wind params
out.wn = GenerateWind(x(start_idx_wind:start_idx_wind+num_windows-1), input.window_lengths);
out.we = GenerateWind(x(start_idx_wind+num_windows:start_idx_wind+2*num_windows-1), input.window_lengths);
if config.force_zero_wd
    out.wd = zeros(sum(input.window_lengths), 1);
else
    out.wd = GenerateWind(x(start_idx_wind+2*num_windows:start_idx_wind+3*num_windows-1), input.window_lengths); 
end

u = out.airspeed_true + input.gyro_z * config.airspeed_offset_y - input.gyro_y * config.airspeed_offset_z;
v_air_body = [ ...
    u'; ...
    (u .* tan(input.slip - b_slip) + config.slip_offset_x * input.gyro_z - config.slip_offset_z * input.gyro_x)'; ...
    (u .* tan(input.aoa - b_aoa) - config.aoa_offset_x * input.gyro_y + config.aoa_offset_y * input.gyro_x)'];

% true airsp vector (assumes zero slip)
out.va_n = sum(squeeze(input.rotm(1,:,:)) .* v_air_body, 1)';
out.va_e = sum(squeeze(input.rotm(2,:,:)) .* v_air_body, 1)';
out.va_d = sum(squeeze(input.rotm(3,:,:)) .* v_air_body, 1)';

% gsp vector
out.gspn = out.va_n + out.wn;
out.gspe = out.va_e + out.we;
out.gspd = out.va_d + out.wd;

out.wind_n = input.gspn - out.va_n;
out.wind_e = input.gspe - out.va_e;
out.wind_d = input.gspd - out.va_d;

% objective
if config.force_zero_wd
    f = [input.gspn - out.gspn; input.gspe - out.gspe; out.wind_d];
else
    f = [input.gspn - out.gspn; input.gspe - out.gspe; input.gspd - out.gspd];
end

if config.use_accz_weighting
   f = f .* [input.weights;input.weights;input.weights]; 
end

end

function [aoa_bias, slip_bias, start_idx_wind] = getAirflowAngleBias(x, config, input, start_idx_bias)
if config.calibration_function == 0
    P0_aoa = x(start_idx_bias+0);
    P1_aoa = x(start_idx_bias+1);
    A_slip = x(start_idx_bias+2);
    B_slip = x(start_idx_bias+3);
    C_slip = x(start_idx_bias+4);
    D_slip = x(start_idx_bias+5);
    
    start_idx_wind = start_idx_bias+5;
    
    aoa_bias = P0_aoa + P1_aoa * (1 + input.acc_z / 9.81);
    slip_bias = A_slip * tanh(B_slip * (input.roll - C_slip)) + D_slip;
elseif config.calibration_function == 1
    P0_aoa = x(start_idx_bias+0);
    P1_aoa = x(start_idx_bias+1);
    P2_aoa = x(start_idx_bias+2);
    P3_aoa = x(start_idx_bias+3);
    P4_aoa = x(start_idx_bias+4);
    P5_aoa = x(start_idx_bias+5);
    if ~config.calibration_use_throttle
        P5_aoa = 0.0;
        x(start_idx_bias+5) = 0.0;
    end

    P0_slip = x(start_idx_bias+6);
    P1_slip = x(start_idx_bias+7);
    P2_slip = x(start_idx_bias+8);
    P3_slip = x(start_idx_bias+9);
    P4_slip = x(start_idx_bias+10);
    P5_slip = x(start_idx_bias+11);
    P6_slip = x(start_idx_bias+12);
    
    aoa_bias = P0_aoa + P1_aoa * input.aoa + P2_aoa * (input.aoa + P3_aoa) .* (input.airspeed + P4_aoa) + P5_aoa * input.throttle;
    slip_bias = P0_slip + P1_slip * (P2_slip * input.airspeed - P3_slip) .* (1.0 + tanh(P4_slip * (input.aoa - P5_slip))) + P6_slip * input.slip;

    start_idx_wind = start_idx_bias+13;

elseif config.calibration_function == 2
    P0_aoa = x(start_idx_bias+0);
    P1_aoa = x(start_idx_bias+1);
    P2_aoa = x(start_idx_bias+2);
    P3_aoa = x(start_idx_bias+3);
    P4_aoa = x(start_idx_bias+4);
    P5_aoa = x(start_idx_bias+5);
    if ~config.calibration_use_throttle
        P5_aoa = 0.0;
        x(start_idx_bias+5) = 0.0;
    end

    P0_slip = x(start_idx_bias+6);
    P1_slip = x(start_idx_bias+7);
    P2_slip = x(start_idx_bias+8);
    P3_slip = x(start_idx_bias+9);
    P4_slip = x(start_idx_bias+10);
    P5_slip = x(start_idx_bias+11);
    P6_slip = x(start_idx_bias+12);
    P7_slip = x(start_idx_bias+13);
    P8_slip = x(start_idx_bias+14);
    P9_slip = x(start_idx_bias+15);
    
    aoa_bias = P0_aoa + P1_aoa * input.aoa + P2_aoa * (input.aoa + P3_aoa) .* (input.airspeed + P4_aoa) + P5_aoa * input.throttle;
    slip_bias = P0_slip + P1_slip * (P2_slip * input.airspeed - P3_slip) .* (1.0 + tanh(P4_slip * (input.aoa - P5_slip))) + P6_slip * input.slip + P7_slip * tanh(P8_slip * (input.roll - P9_slip));

    start_idx_wind = start_idx_bias+16;
else
    error('Unknown calibration function')
end
end

end