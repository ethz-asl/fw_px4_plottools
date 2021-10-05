% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
% Airspeed scale solver / / / / / / / / / / / / / / / / / / / / / / / / / /
function [xopt, opt_info, mean_gsp_err, std_gsp_err] = SolveAirspeedScale(sysvector, x0, lb, ub, config)

dt_rs = 0.05;
time_resampled = config.t_st_cal:dt_rs:config.t_ed_cal;
dp = resample(sysvector.differential_pressure_0.differential_pressure_raw_pa, time_resampled);
baro = resample(sysvector.sensor_baro_0.pressure * 100, time_resampled);
temp = resample(sysvector.differential_pressure_0.temperature, time_resampled);
q_0 = resample(sysvector.vehicle_attitude_0.q_0, time_resampled);
q_1 = resample(sysvector.vehicle_attitude_0.q_1, time_resampled);
q_2 = resample(sysvector.vehicle_attitude_0.q_2, time_resampled);
q_3 = resample(sysvector.vehicle_attitude_0.q_3, time_resampled);
[yaw, ~, ~] = quat2angle([q_0.Data, q_1.Data, q_2.Data, q_3.Data]);
gspn = resample(sysvector.vehicle_gps_position_0.vel_n_m_s, time_resampled);
gspe = resample(sysvector.vehicle_gps_position_0.vel_e_m_s, time_resampled);
airsp = resample(sysvector.airspeed_0.true_airspeed_m_s, time_resampled);

[xopt,resnorm,residual,exitflag,output] = ...
    lsqnonlin(@(x)Dp2Sp(x,dp.Data,baro.Data,temp.Data,yaw,gspn.Data,gspe.Data, config), x0, lb, ub);

opt_info.resnorm = resnorm;
opt_info.residual = residual;
opt_info.exitflag = exitflag;
opt_info.output = output;

% calculate outputs
[~,out] = Dp2Sp(xopt,dp.Data,baro.Data,temp.Data,yaw,gspn.Data,gspe.Data, config);

% mean / std of ground speed errors
mean_gsp_err = mean([gspn.Data; gspe.Data] - [out(:,3); out(:,4)]);
std_gsp_err = std([gspn.Data; gspe.Data] - [out(:,3); out(:,4)]);

%% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / 
% Output function / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

wind_n_raw = gspn.Data - airsp.Data .* cos(yaw);
wind_e_raw = gspe.Data - airsp.Data .* sin(yaw);

figure('color','w');

lines_ = lines(7);
plot_opacity = 0.5;

result_plots(1) = subplot(2,1,1); hold on; grid on; box on;
plot(time_resampled, wind_n_raw);
plot(time_resampled, wind_e_raw);
plot(time_resampled, ones(length(time_resampled),1)*xopt(1), 'linewidth', 2);
plot(time_resampled, ones(length(time_resampled),1)*xopt(2), 'linewidth', 2);
legend('w_n raw','w_e raw','w_n fit','w_e fit');
ylabel('wind sp. [m/s]');

result_plots(2) = subplot(2,1,2); hold on; grid on; box on;
plot(time_resampled, sqrt(gspn.Data.^2 + gspe.Data.^2), '-.', 'color', lines_(1,:) * plot_opacity + ones(1,3) * (1 - plot_opacity));
plot(time_resampled, sqrt(out(:,3).^2 + out(:,4).^2), 'color', lines_(1,:));
plot(time_resampled, sqrt(wind_n_raw.^2 + wind_e_raw.^2), '-.', 'color', lines_(2,:) * plot_opacity + ones(1,3) * (1 - plot_opacity));
plot(time_resampled, sqrt(ones(length(time_resampled),1)*xopt(1).^2 + ones(length(time_resampled),1)*xopt(2).^2), 'color', lines_(2,:));
plot(time_resampled, airsp.Data, '-.', 'color', lines_(3,:) * plot_opacity + ones(1,3) * (1 - plot_opacity));
plot(time_resampled, sqrt(out(:,1).^2 + out(:,2).^2), 'color', lines_(3,:));

legend('gsp. data','gsp. fit','wind data','wind fit','airsp. data','airsp. cal');
ylabel('speeds [m/s]');

xlabel('Time [s]');
linkaxes(result_plots(:),'x');
xlim(result_plots(:), time_resampled([1 end]));

% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / 
% Output function / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
function [f, out] = Dp2Sp(x,dp_data,baro_data,temp_data,yaw_data,gspn_data,gspe_data, config)

% current solution
wn = x(1);
we = x(2);
sf = x(3);

config.airspeed_scale_factor = sf;

[airspeed_true, ~] = CalculateAirspeed(dp_data, baro_data, temp_data, config);

% true airsp vector (assumes zero slip)
va_n = airspeed_true.*cos(yaw_data);
va_e = airspeed_true.*sin(yaw_data);

% gsp vector
gsp_n = va_n + wn;
gsp_e = va_e + we;

% objective
f = [gspn_data - gsp_n; gspe_data - gsp_e];

% outputs
out = [va_n, va_e, gsp_n, gsp_e];