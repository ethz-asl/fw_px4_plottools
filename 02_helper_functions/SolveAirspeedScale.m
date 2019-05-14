% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
% Airspeed scale solver / / / / / / / / / / / / / / / / / / / / / / / / / /
function [xopt, opt_info, mean_gsp_err, std_gsp_err] = SolveAirspeedScale(sysvector, tube_dia, tube_len, pitot_type, tspan, x0, lb, ub)

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
    lsqnonlin(@(x)Dp2Sp(x,dp.Data,baro.Data,temp.Data,yaw,gspn.Data,gspe.Data, ...
    tube_dia,tube_len,pitot_type), x0, lb, ub);

opt_info.resnorm = resnorm;
opt_info.residual = residual;
opt_info.exitflag = exitflag;
opt_info.output = output;

% calculate outputs
[~,out] = Dp2Sp(xopt,dp.Data,baro.Data,temp.Data,yaw,gspn.Data,gspe.Data, ...
    tube_dia,tube_len,pitot_type);

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
function [f, out] = Dp2Sp(x,dp_data,baro_data,temp_data,yaw_data,gspn_data,gspe_data, ...
    tube_dia,tube_len,pitot_type)

% current solution
wn = x(1);
we = x(2);
sf = x(3);

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
airspeed_indicated = sqrt(2.0*abs(dp_corr) / 1.225) * sf;
airspeed_indicated(dp_corr < 0.0) = 0.0;

% compute the true airspeed from the indicated airspeed
airspeed_true = airspeed_indicated .* sqrt(1.225 ./ rho_air);

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