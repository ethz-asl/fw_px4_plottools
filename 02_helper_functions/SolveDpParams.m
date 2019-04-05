function [xopt, opt_info] = SolveDpParams(sysvector, tspan, x0, lb, ub)

dt_rs = 0.05;
time_resampled = tspan(1):dt_rs:tspan(2);
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
    lsqnonlin(@(x)Dp2Sp(x,dp.Data,baro.Data,temp.Data,yaw,gspn.Data,gspe.Data), ...
    x0, lb, ub);

opt_info.resnorm = resnorm;
opt_info.residual = residual;
opt_info.exitflag = exitflag;
opt_info.output = output;

%% plot results

[~,out] = Dp2Sp(xopt,dp.Data,baro.Data,temp.Data,yaw,gspn.Data,gspe.Data);

wind_n_raw = gspn.Data - airsp.Data .* cos(yaw);
wind_e_raw = gspe.Data - airsp.Data .* sin(yaw);

figure('color','w');

result_plots(1) = subplot(2,1,1); hold on; grid on; box on;
plot(time_resampled, wind_n_raw);
plot(time_resampled, wind_e_raw);
plot(time_resampled, ones(length(time_resampled),1)*xopt(3), 'linewidth', 2);
plot(time_resampled, ones(length(time_resampled),1)*xopt(4), 'linewidth', 2);
legend('w_n raw','w_e raw','w_n cal','w_e cal');
ylabel('wind sp. [m/s]');

result_plots(2) = subplot(2,1,2); hold on; grid on; box on;
plot(time_resampled, sqrt(gspn.Data.^2 + gspe.Data.^2));
plot(time_resampled, sqrt(out(:,3).^2 + out(:,4).^2), 'linewidth', 2);
plot(time_resampled, sqrt(wind_n_raw.^2 + wind_e_raw.^2));
plot(time_resampled, sqrt(ones(length(time_resampled),1)*xopt(3).^2 + ones(length(time_resampled),1)*xopt(4).^2), 'linewidth', 2);
plot(time_resampled, airsp.Data);
plot(time_resampled, sqrt(out(:,1).^2 + out(:,2).^2), 'linewidth', 2);

legend('gsp. data','gsp. cal','wind raw','wind cal','airsp. uncal','airsp. cal');
ylabel('speeds [m/s]');

xlabel('Time [s]');
linkaxes(result_plots(:),'x');
xlim(result_plots(:), time_resampled([1 end]));
