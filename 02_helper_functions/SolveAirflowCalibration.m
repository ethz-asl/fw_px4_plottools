% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
% Airflow Calibration solver / / / / / / / / / / / / / / / / / / / / / / / / / /
function [xopt, opt_info, mean_gsp_err, std_gsp_err, mean_wind_d] = SolveAirflowCalibration(sysvector, tube_dia, tube_len, pitot_type, tspan, x0, lb, ub, force_zero_wd, pitching_calibration, slip_bias, aoa_mounting_offset, slip_mounting_offset)

dt_rs = 0.05;
time_resampled = tspan(1):dt_rs:tspan(2);
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
airsp = resample(sysvector.airspeed_0.true_airspeed_m_s, time_resampled);
aoa = resample(sysvector.airflow_aoa_0.aoa_rad, time_resampled);
slip = resample(sysvector.airflow_slip_0.slip_rad, time_resampled);
gyro_x = resample(sysvector.sensor_gyro_0.x, time_resampled);
gyro_y = resample(sysvector.sensor_gyro_0.y, time_resampled);
gyro_z = resample(sysvector.sensor_gyro_0.z, time_resampled);

if pitching_calibration
    [~,out_before] = Dp2SpAnglesVertical(x0,dp.Data,baro.Data,temp.Data,rotm,gspn.Data,gspe.Data,gspd.Data, ...
        aoa.Data,slip.Data,gyro_x.Data,gyro_y.Data,gyro_z.Data,tube_dia,tube_len,pitot_type,slip_bias,aoa_mounting_offset,slip_mounting_offset);
    
    [xopt,resnorm,residual,exitflag,output] = ...
        lsqnonlin(@(x)Dp2SpAnglesVertical(x,dp.Data,baro.Data,temp.Data,rotm,gspn.Data,gspe.Data,gspd.Data, ...
        aoa.Data,slip.Data,gyro_x.Data,gyro_y.Data,gyro_z.Data,tube_dia,tube_len,pitot_type,slip_bias,aoa_mounting_offset,slip_mounting_offset), x0, lb, ub);
else
    [~,out_before] = Dp2SpAngles(x0,dp.Data,baro.Data,temp.Data,rotm,gspn.Data,gspe.Data,gspd.Data, ...
        aoa.Data,slip.Data,gyro_x.Data,gyro_y.Data,gyro_z.Data,tube_dia,tube_len,pitot_type,force_zero_wd,aoa_mounting_offset,slip_mounting_offset);
    
    [xopt,resnorm,residual,exitflag,output] = ...
        lsqnonlin(@(x)Dp2SpAngles(x,dp.Data,baro.Data,temp.Data,rotm,gspn.Data,gspe.Data,gspd.Data, ...
        aoa.Data,slip.Data,gyro_x.Data,gyro_y.Data,gyro_z.Data,tube_dia,tube_len,pitot_type,force_zero_wd,aoa_mounting_offset,slip_mounting_offset), x0, lb, ub);
end

opt_info.resnorm = resnorm;
opt_info.residual = residual;
opt_info.exitflag = exitflag;
opt_info.output = output;

% calculate outputs
if pitching_calibration
    [~,out] = Dp2SpAnglesVertical(xopt,dp.Data,baro.Data,temp.Data,rotm,gspn.Data,gspe.Data,gspd.Data, ...
        aoa.Data,slip.Data,gyro_x.Data,gyro_y.Data,gyro_z.Data,tube_dia,tube_len,pitot_type,slip_bias,aoa_mounting_offset,slip_mounting_offset);
else
    [~,out] = Dp2SpAngles(xopt,dp.Data,baro.Data,temp.Data,rotm,gspn.Data,gspe.Data,gspd.Data, ...
        aoa.Data,slip.Data,gyro_x.Data,gyro_y.Data,gyro_z.Data,tube_dia,tube_len,pitot_type,force_zero_wd,aoa_mounting_offset,slip_mounting_offset);
end

% mean / std of ground speed errors
mean_gsp_err = mean([gspn.Data; gspe.Data] - [out(:,4); out(:,5)]);
std_gsp_err = std([gspn.Data; gspe.Data] - [out(:,4); out(:,5)]);
mean_wind_d = mean(out(:,7));

%% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / 
% Output function / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
lines_ = lines(7);
plot_opacity = 0.5;

if pitching_calibration
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
end

xlabel('Time [s]');
linkaxes(result_plots(:),'x');
xlim(result_plots(:), time_resampled([1 end]));


% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / 
% Output function / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
function [f, out] = Dp2SpAngles(x,dp_data,baro_data,temp_data,rotm_data,gspn_data,gspe_data,gspd_data, ...
    aoa_data, slip_data, gyro_x_data, gyro_y_data, gyro_z_data, tube_dia,tube_len,pitot_type,force_zero_wd,aoa_mounting_offset,slip_mounting_offset)

% current solution
wn = x(1);
we = x(2);
sf = x(3);
ba = x(4);
bs = x(5);
if force_zero_wd
    wd = 0;
else
    wd = x(6); 
end

% set negative dp to 0
dp_data(dp_data<0) = 0;
    
% density of the air
rho_air = (baro_data) ./ (287.1 .* (273.15 + temp_data));

% correct differential pressure for tube/pitot lossesd
if (pitot_type == 0)
    % drotek pitot
    % use tube_len
    
    dp_corr = dp_data * 96600.0 ./ baro_data;
    
    % flow through sensor
    flow_SDP33 = (300.805 - 300.878 ./ (0.00344205 * dp_corr.^0.68698 + 1.0)) .* 1.29 ./ rho_air;

    % for too small readings the compensation might result in a negative flow which causes numerical issues
    if (flow_SDP33 < 0.0)
        flow_SDP33 = 0.0;
    end

    dp_pitot = (0.0032 * flow_SDP33 .* flow_SDP33 + 0.0123 * flow_SDP33 + 1.0) .* 1.29 ./ rho_air;

    % pressure drop through tube
    dp_tube = (flow_SDP33 * 0.674) / 450.0 * tube_len .* rho_air / 1.29;

    % speed at pitot-tube tip due to flow through sensor
    dv = 0.125 * flow_SDP33;

    % sum of all pressure drops
    dp_corr = dp_corr + dp_tube + dp_pitot;
    
elseif (pitot_type == 1)
    % custom pitot
    % use tube_dia + tube_len

    % DP reading of the sensor (101 for SDP3X, 62 for SDP600)
    dpSensor = 101; % [Pa]

    % Massflow (4.79e-7 for SDP3X, 6.17e-7 for SDP600)
    massflow = 4.79e-7; % [kg/s]

    % compute correction factor
    d_pow4 = tube_dia^4;

    % viscosity of the air
    vis_air = (18.205 + 0.0484 * (temp_data - 20.0)) * 1e-6;
    
    % denominator
    denominator = pi * d_pow4 * rho_air .* dp_data;
    idx = find(abs(denominator)>1e-32);

    eps = zeros(size(denominator));
    eps(idx) =  -64.0 * tube_len * massflow * vis_air(idx) .* (sqrt(1.0 + 8.0 * dp_data(idx) / dpSensor)-1.0)./denominator(idx);

    % limit eps
    eps(eps>=1.0) = 0.0;
    eps(eps<=-1.0) = 0.0;

    % differential pressure corrected for tube losses
    dp_corr = dp_data ./ (1 + eps);
    
    % speed at pitot-tube tip due to flow through sensor
    dv = 0;
end

% compute indicated airspeed
airspeed_indicated = (sqrt(2.0*abs(dp_corr) / 1.225) + dv) * sf;
airspeed_indicated(dp_corr < 0.0) = 0.0;

% compute the true airspeed from the indicated airspeed
airspeed_true = airspeed_indicated .* sqrt(1.225 ./ rho_air);

v_air_body = [ ...
    airspeed_true'; ...
    (airspeed_true .* tan(slip_data - bs) - slip_mounting_offset(1) * gyro_z_data + slip_mounting_offset(2) * gyro_x_data)'; ...
    (airspeed_true .* tan(aoa_data - ba) + aoa_mounting_offset(1) * gyro_y_data - aoa_mounting_offset(2) * gyro_x_data)'];

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
if force_zero_wd
    f = [gspn_data - gsp_n; gspe_data - gsp_e; wind_d];
else
    f = [gspn_data - gsp_n; gspe_data - gsp_e; gspd_data - gsp_d];
end

% outputs
out = [va_n, va_e, va_d, gsp_n, gsp_e, gsp_d, wind_d, airspeed_true];


% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / 
% Output function / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
function [f, out] = Dp2SpAnglesVertical(x,dp_data,baro_data,temp_data,rotm_data,gspn_data,gspe_data,gspd_data, ...
    aoa_data, slip_data, gyro_x_data, gyro_y_data, gyro_z_data, tube_dia,tube_len,pitot_type,slip_bias,aoa_mounting_offset,slip_mounting_offset)

% current solution

sf = x(1);
ba = x(2);
wd = x(3);
bs = slip_bias;

% set negative dp to 0
dp_data(dp_data<0) = 0;
    
% density of the air
rho_air = (baro_data) ./ (287.1 .* (273.15 + temp_data));

% correct differential pressure for tube/pitot lossesd
if (pitot_type == 0)
    % drotek pitot
    % use tube_len
    
    dp_corr = dp_data * 96600.0 ./ baro_data;
    
    % flow through sensor
    flow_SDP33 = (300.805 - 300.878 ./ (0.00344205 * dp_corr.^0.68698 + 1.0)) .* 1.29 ./ rho_air;

    % for too small readings the compensation might result in a negative flow which causes numerical issues
    if (flow_SDP33 < 0.0)
        flow_SDP33 = 0.0;
    end

    dp_pitot = (0.0032 * flow_SDP33 .* flow_SDP33 + 0.0123 * flow_SDP33 + 1.0) .* 1.29 ./ rho_air;

    % pressure drop through tube
    dp_tube = (flow_SDP33 * 0.674) / 450.0 * tube_len .* rho_air / 1.29;

    % speed at pitot-tube tip due to flow through sensor
    dv = 0.125 * flow_SDP33;

    % sum of all pressure drops
    dp_corr = dp_corr + dp_tube + dp_pitot;
    
elseif (pitot_type == 1)
    % custom pitot
    % use tube_dia + tube_len

    % DP reading of the sensor (101 for SDP3X, 62 for SDP600)
    dpSensor = 101; % [Pa]

    % Massflow (4.79e-7 for SDP3X, 6.17e-7 for SDP600)
    massflow = 4.79e-7; % [kg/s]

    % compute correction factor
    d_pow4 = tube_dia^4;

    % viscosity of the air
    vis_air = (18.205 + 0.0484 * (temp_data - 20.0)) * 1e-6;
    
    % denominator
    denominator = pi * d_pow4 * rho_air .* dp_data;
    idx = find(abs(denominator)>1e-32);

    eps = zeros(size(denominator));
    eps(idx) =  -64.0 * tube_len * massflow * vis_air(idx) .* (sqrt(1.0 + 8.0 * dp_data(idx) / dpSensor)-1.0)./denominator(idx);

    % limit eps
    eps(eps>=1.0) = 0.0;
    eps(eps<=-1.0) = 0.0;

    % differential pressure corrected for tube losses
    dp_corr = dp_data ./ (1 + eps);
    
    % speed at pitot-tube tip due to flow through sensor
    dv = 0;
end

% compute indicated airspeed
airspeed_indicated = (sqrt(2.0*abs(dp_corr) / 1.225) + dv) * sf;
airspeed_indicated(dp_corr < 0.0) = 0.0;

% compute the true airspeed from the indicated airspeed
airspeed_true = airspeed_indicated .* sqrt(1.225 ./ rho_air);

v_air_body = [ ...
    airspeed_true'; ...
    (airspeed_true .* tan(slip_data - bs) - slip_mounting_offset(1) * gyro_z_data + slip_mounting_offset(2) * gyro_x_data)'; ...
    (airspeed_true .* tan(aoa_data - ba) + aoa_mounting_offset(1) * gyro_y_data - aoa_mounting_offset(2) * gyro_x_data)'];

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