% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
% Airflow Calibration solver / / / / / / / / / / / / / / / / / / / / / / / / / /
function [xopt, opt_info, mean_gsp_err, std_gsp_err, mean_wind_d, loiter_data] = SolveAirflowCalibration(sysvector, topics, paramvector, params, x0, lb, ub, config)

dt_rs = 0.05;
time_resampled = config.t_st_cal:dt_rs:config.t_ed_cal;
dp = resample(sysvector.differential_pressure_0.differential_pressure_raw_pa, time_resampled);
baro = resample(sysvector.sensor_baro_0.pressure * 100, time_resampled);
temp = resample(sysvector.differential_pressure_0.temperature, time_resampled);
q_0 = resample(sysvector.vehicle_attitude_0.q_0, time_resampled);
q_1 = resample(sysvector.vehicle_attitude_0.q_1, time_resampled);
q_2 = resample(sysvector.vehicle_attitude_0.q_2, time_resampled);
q_3 = resample(sysvector.vehicle_attitude_0.q_3, time_resampled);
rotm = quat2rotm([q_0.Data, q_1.Data, q_2.Data, q_3.Data]);
gspn = resample(sysvector.vehicle_gps_position_0.vel_n_m_s, time_resampled);
gspe = resample(sysvector.vehicle_gps_position_0.vel_e_m_s, time_resampled);
gspd = resample(sysvector.vehicle_gps_position_0.vel_d_m_s, time_resampled);
gyro_x = resample(sysvector.sensor_gyro_0.x, time_resampled);
gyro_y = resample(sysvector.sensor_gyro_0.y, time_resampled);
gyro_z = resample(sysvector.sensor_gyro_0.z, time_resampled);
acc_z = resample(sysvector.sensor_accel_0.z, time_resampled);
if topics.actuator_controls_0.logged
    throttle = resample(sysvector.actuator_controls_0_0.control_3, time_resampled);
end


[aoa, slip, aoa_logged, slip_logged] = CalibrateAirflowAngles(sysvector, topics, paramvector, params, config);
if (~aoa_logged || ~slip_logged)
    error('Slip and AoA measurements are required')
end
aoa = resample(aoa, time_resampled);
slip = resample(slip, time_resampled);


if config.verbose
    options = optimoptions(@lsqnonlin,'Display','final');
else
    options = optimoptions(@lsqnonlin,'Display','off');
end

if config.pitching_calibration
    [~,out_before] = Dp2SpAnglesVertical(x0,dp.Data,baro.Data,temp.Data,rotm,gspn.Data,gspe.Data,gspd.Data, ...
        aoa.Data,slip.Data,gyro_x.Data,gyro_y.Data,gyro_z.Data,config);
    
    [xopt,resnorm,residual,exitflag,output] = ...
        lsqnonlin(@(x)Dp2SpAnglesVertical(x,dp.Data,baro.Data,temp.Data,rotm,gspn.Data,gspe.Data,gspd.Data, ...
        aoa.Data,slip.Data,gyro_x.Data,gyro_y.Data,gyro_z.Data,config), x0, lb, ub, options);
else
    [~,out_before] = Dp2SpAngles(x0,dp.Data,baro.Data,temp.Data,rotm,gspn.Data,gspe.Data,gspd.Data, ...
        aoa.Data,slip.Data,gyro_x.Data,gyro_y.Data,gyro_z.Data,config);
    
    [xopt,resnorm,residual,exitflag,output] = ...
        lsqnonlin(@(x)Dp2SpAngles(x,dp.Data,baro.Data,temp.Data,rotm,gspn.Data,gspe.Data,gspd.Data, ...
        aoa.Data,slip.Data,gyro_x.Data,gyro_y.Data,gyro_z.Data,config), x0, lb, ub, options);
end

opt_info.resnorm = resnorm;
opt_info.residual = residual;
opt_info.exitflag = exitflag;
opt_info.output = output;

% calculate outputs
if config.pitching_calibration
    [~,out] = Dp2SpAnglesVertical(xopt,dp.Data,baro.Data,temp.Data,rotm,gspn.Data,gspe.Data,gspd.Data, ...
        aoa.Data,slip.Data,gyro_x.Data,gyro_y.Data,gyro_z.Data,config);
else
    [~,out] = Dp2SpAngles(xopt,dp.Data,baro.Data,temp.Data,rotm,gspn.Data,gspe.Data,gspd.Data, ...
        aoa.Data,slip.Data,gyro_x.Data,gyro_y.Data,gyro_z.Data,config);
end

% mean / std of ground speed errors
mean_gsp_err = mean([gspn.Data; gspe.Data] - [out(:,4); out(:,5)]);
std_gsp_err = std([gspn.Data; gspe.Data] - [out(:,4); out(:,5)]);
mean_wind_d = mean(out(:,7));

%% Get loiter characteristics
[pitch, roll, yaw] = QuaternionToEuler(q_0, q_1, q_2, q_3);
loiter_data.roll = mean(roll);
loiter_data.airspeed = mean(sqrt(out(:,1).^2+out(:,2).^2+out(:,3).^2));
loiter_data.g_load = mean(acc_z.Data/9.81);
if topics.actuator_controls_0.logged
    loiter_data.throttle = mean(throttle.Data);
else
    loiter_data.throttle = 0.0;
end
if config.pitching_calibration
    aoa_bias = xopt(2);
    slip_bias = deg2rad(config.slip_bias_deg);
    scale_factor = xopt(1);
    w_e_est = 0.0;
    w_n_est = 0.0;
    w_d_est = xopt(3);
else
    aoa_bias = xopt(4);
    slip_bias = xopt(5);
    scale_factor = xopt(3);
    w_e_est = xopt(2);
    w_n_est = xopt(1);
    w_d_est = mean_wind_d;
end
loiter_data.slip = mean(slip.Data) - slip_bias;
loiter_data.uncorrected_slip = mean(slip.Data);
loiter_data.aoa = mean(aoa.Data) - aoa_bias;
loiter_data.uncorrected_aoa = mean(aoa.Data);
loiter_data.aoa_bias = aoa_bias;
loiter_data.slip_bias = slip_bias;
loiter_data.scale_factor = scale_factor;
loiter_data.we = w_e_est;
loiter_data.wn = w_n_est;
loiter_data.wd = w_d_est;

%% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / 
% Output function / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
if config.verbose
    lines_ = lines(7);
    plot_opacity = 0.5;

    if config.pitching_calibration
        wind_d_raw = out_before(:,7);
        wind_d_fit = out(:,7);
        wind_d_fit_const = xopt(3);

        figure('color','w');

        result_plots(1) = subplot(2,1,1); hold on; grid on; box on;
        plot(time_resampled, wind_d_raw);
        plot(time_resampled, wind_d_fit, 'linewidth', 2);
        plot(time_resampled, ones(length(time_resampled),1)*wind_d_fit_const, 'linewidth', 2);
        legend('w_d raw','w_d fit', 'w_d fit const');
        ylabel('wind sp. [m/s]');

        result_plots(2) = subplot(2,1,2); hold on; grid on; box on;
        plot(time_resampled, out_before(:,6), '-.', 'color', lines_(1,:) * plot_opacity + ones(1,3) * (1 - plot_opacity));
        plot(time_resampled, out(:,6), '-.', 'color', lines_(1,:));
        plot(time_resampled, out_before(:,8), '-.', 'color', lines_(3,:) * plot_opacity + ones(1,3) * (1 - plot_opacity));
        plot(time_resampled, out(:,8), 'color', lines_(3,:));

        legend('gspd_d raw','gspd_d fit','airsp. data','airsp. cal');
        ylabel('speeds [m/s]');
        xlabel('Time [s]');
        linkaxes(result_plots(:),'x');
        xlim(result_plots(:), time_resampled([1 end]));

    else
        wind_n_raw = gspn.Data - out_before(:,1);
        wind_e_raw = gspe.Data - out_before(:,2);
        wind_d_raw = out_before(:,7);

        figure('color','w');

        result_plots(1) = subplot(2,1,1); hold on; grid on; box on;
        plot(time_resampled, wind_n_raw);
        plot(time_resampled, wind_e_raw);
        plot(time_resampled, wind_d_raw);
        plot(time_resampled, ones(length(time_resampled),1)*xopt(1), 'linewidth', 2);
        plot(time_resampled, ones(length(time_resampled),1)*xopt(2), 'linewidth', 2);
        plot(time_resampled, out(:,6), 'linewidth', 2);
        legend('w_n raw','w_e raw', 'w_d raw','w_n fit','w_e fit', 'w_d fit');
        ylabel('wind sp. [m/s]');

        result_plots(2) = subplot(2,1,2); hold on; grid on; box on;
        plot(time_resampled, sqrt(gspn.Data.^2 + gspe.Data.^2), '-.', 'color', lines_(1,:) * plot_opacity + ones(1,3) * (1 - plot_opacity));
        plot(time_resampled, sqrt(out(:,4).^2 + out(:,5).^2), 'color', lines_(1,:));
        plot(time_resampled, sqrt(wind_n_raw.^2 + wind_e_raw.^2), '-.', 'color', lines_(2,:) * plot_opacity + ones(1,3) * (1 - plot_opacity));
        plot(time_resampled, sqrt(ones(length(time_resampled),1)*xopt(1).^2 + ones(length(time_resampled),1)*xopt(2).^2), 'color', lines_(2,:));
        plot(time_resampled, out_before(:,8), '-.', 'color', lines_(3,:) * plot_opacity + ones(1,3) * (1 - plot_opacity));
        plot(time_resampled, out(:,8), 'color', lines_(3,:));

        legend('gsp. data','gsp. fit','wind data','wind fit','airsp. data','airsp. cal');
        ylabel('speeds [m/s]');
        xlabel('Time [s]');
        linkaxes(result_plots(:),'x');
        xlim(result_plots(:), time_resampled([1 end]));

        figure('color','w');
        validation_plots(1) = subplot(2,1,1); hold on; grid on; box on;
        plot(time_resampled, gspn.Data - out(:,1), 'color', lines_(1,:));
        plot(time_resampled, gspe.Data - out(:,2), 'color', lines_(2,:));
        plot(time_resampled, yaw.Data, 'color', lines_(3,:));
        legend('w_n opt','w_e opt', 'yaw');
        ylabel('speeds | angles [m/s | deg]');

        validation_plots(2) = subplot(2,1,2); hold on; grid on; box on;
        plot(time_resampled, gspd.Data - out(:,3), 'color', lines_(1,:));
        plot(time_resampled, gspd.Data, 'color', lines_(2,:));
        plot(time_resampled, rad2deg(pitch.Data), 'color', lines_(3,:));
        ylabel('[m/s | deg | m]');
        legend('w_d opt','groundspeed down','pitch');


        xlabel('Time [s]');
        linkaxes(validation_plots(:),'x');
        xlim(validation_plots(:), time_resampled([1 end]));

    end
end


% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / 
% Output function / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
function [f, out] = Dp2SpAngles(x,dp_data,baro_data,temp_data,rotm_data,gspn_data,gspe_data,gspd_data, ...
    aoa_data, slip_data, gyro_x_data, gyro_y_data, gyro_z_data, config)

% current solution
wn = x(1);
we = x(2);
sf = x(3);
ba = x(4);
bs = x(5);
if config.force_zero_wd
    wd = 0;
else
    wd = x(6); 
end

config.airspeed_scale_factor = sf;

[airspeed_true, ~] = CalculateAirspeed(dp_data, baro_data, temp_data, config);

v_air_body = [ ...
    airspeed_true'; ...
    (airspeed_true .* tan(slip_data - bs) + config.slip_offset_x * gyro_z_data - config.slip_offset_z * gyro_x_data)'; ...
    (airspeed_true .* tan(aoa_data - ba) - config.aoa_offset_x * gyro_y_data + config.aoa_offset_y * gyro_x_data)'];

% true airsp vector (assumes zero slip)
va_n = sum(squeeze(rotm_data(1,:,:)) .* v_air_body, 1)';
va_e = sum(squeeze(rotm_data(2,:,:)) .* v_air_body, 1)';
va_d = sum(squeeze(rotm_data(3,:,:)) .* v_air_body, 1)';

% gsp vector
gsp_n = va_n + wn;
gsp_e = va_e + we;
gsp_d = va_d + wd;

wind_d = gspd_data - va_d;

% objective
if config.force_zero_wd
    f = [gspn_data - gsp_n; gspe_data - gsp_e; wind_d];
else
    f = [gspn_data - gsp_n; gspe_data - gsp_e; gspd_data - gsp_d];
end

% outputs
out = [va_n, va_e, va_d, gsp_n, gsp_e, gsp_d, wind_d, airspeed_true];


% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / 
% Output function / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
function [f, out] = Dp2SpAnglesVertical(x,dp_data,baro_data,temp_data,rotm_data,gspn_data,gspe_data,gspd_data, ...
    aoa_data, slip_data, gyro_x_data, gyro_y_data, gyro_z_data, config)

% current solution

sf = x(1);
ba = x(2);
wd = x(3);
bs = deg2rad(config.slip_bias_deg);
config.airspeed_scale_factor = sf;

[airspeed_true, ~] = CalculateAirspeed(dp_data, baro_data, temp_data, config);

v_air_body = [ ...
    airspeed_true'; ...
    (airspeed_true .* tan(slip_data - bs) + config.slip_offset_x * gyro_z_data - config.slip_offset_z * gyro_x_data)'; ...
    (airspeed_true .* tan(aoa_data - ba) - config.aoa_offset_x * gyro_y_data + config.aoa_offset_y * gyro_x_data)'];

% true airsp vector (assumes zero slip)
va_n = sum(squeeze(rotm_data(1,:,:)) .* v_air_body, 1)';
va_e = sum(squeeze(rotm_data(2,:,:)) .* v_air_body, 1)';
va_d = sum(squeeze(rotm_data(3,:,:)) .* v_air_body, 1)';

% gsp vector
gsp_d = va_d + wd;

wind_d = gspd_data - va_d;

% objective
f = [gspd_data - gsp_d, wind_d - wd];

% outputs
out = [va_n, va_e, va_d, gspn_data, gspe_data, gsp_d, wind_d, airspeed_true];