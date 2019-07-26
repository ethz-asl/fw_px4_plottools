% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
% Airspeed scale solver / / / / / / / / / / / / / / / / / / / / / / / / / /
function [xopt, opt_info, mean_gsp_err, std_gsp_err] = SolveVaneOffsets(sysvector, topics, paramvector, tspan, x0, lb, ub)


% calibrated airflow angle measurements
if topics.airflow_aoa.logged && topics.airflow_slip.logged
    aoa_meas = sysvector.airflow_aoa_0.aoa_rad;
    slip_meas = sysvector.airflow_slip_0.slip_rad;

else
    % calibrated airflow angle measurements
    for i = 0:topics.sensor_hall.num_instances - 1
        hall_name = strcat('sensor_hall_', num2str(i));

        if sysvector.(hall_name).instance.Data(1) == paramvector.cal_av_aoa_id.Data(1)
            aoa_hall_data = sysvector.(hall_name);
        end

        if sysvector.(hall_name).instance.Data(1) == paramvector.cal_av_slip_id.Data(1)
            slip_hall_data = sysvector.(hall_name);
        end
    end

    aoa_meas = timeseries(deg2rad(1e-7 * paramvector.cal_av_aoa_rev.Data(1) * ...
            (paramvector.cal_av_aoa_p0.Data(1) + ...
            paramvector.cal_av_aoa_p1.Data(1) .* aoa_hall_data.mag_T.Data + ...
            paramvector.cal_av_aoa_p2.Data(1) .* aoa_hall_data.mag_T.Data .* aoa_hall_data.mag_T.Data + ...
            paramvector.cal_av_aoa_p3.Data(1) .* aoa_hall_data.mag_T.Data .* aoa_hall_data.mag_T.Data .* aoa_hall_data.mag_T.Data)),...
            aoa_hall_data.mag_T.Time);
    slip_meas = timeseries(deg2rad(1e-7 * paramvector.cal_av_slip_rev.Data(1) * ...
            (paramvector.cal_av_slip_p0.Data(1) + ...
            paramvector.cal_av_slip_p1.Data(1) .* slip_hall_data.mag_T.Data + ...
            paramvector.cal_av_slip_p2.Data(1) .* slip_hall_data.mag_T.Data .* slip_hall_data.mag_T.Data + ...
            paramvector.cal_av_slip_p3.Data(1) .* slip_hall_data.mag_T.Data .* slip_hall_data.mag_T.Data .* slip_hall_data.mag_T.Data)),...
            slip_hall_data.Time);
end

% synchronise the data
dt_rs = 0.05;
time_resampled = tspan(1):dt_rs:tspan(2);
len_t = length(time_resampled);

% resample inputs / outputs
aoa_meas = resample(aoa_meas, time_resampled);
slip_meas = resample(slip_meas, time_resampled);
vel_n = resample(sysvector.vehicle_gps_position_0.vel_n_m_s, time_resampled);
vel_e = resample(sysvector.vehicle_gps_position_0.vel_e_m_s, time_resampled);
vel_d = resample(sysvector.vehicle_gps_position_0.vel_d_m_s, time_resampled);
airspeed = resample(sysvector.airspeed_0.true_airspeed_m_s, time_resampled);
q_0 = resample(sysvector.vehicle_attitude_0.q_0, time_resampled);
q_1 = resample(sysvector.vehicle_attitude_0.q_1, time_resampled);
q_2 = resample(sysvector.vehicle_attitude_0.q_2, time_resampled);
q_3 = resample(sysvector.vehicle_attitude_0.q_3, time_resampled);

% construct body to inertial transform
Hi2b = quat2dcm([q_0.Data, q_1.Data, q_2.Data, q_3.Data]);
Hb2i_rows = zeros(len_t, 9);
for i = 1:len_t
    htemp = Hi2b(:,:,i)';
    Hb2i_rows(i,:) =  [htemp(1,:), htemp(2,:), htemp(3,:)];
end

% solve nonlinear least squares
[xopt,resnorm,residual,exitflag,output] = ...
    lsqnonlin(@(x)SolveVaneMountingBias(x, vel_n.Data, vel_e.Data, vel_d.Data, ...
    aoa_meas.Data, slip_meas.Data, airspeed.Data, Hb2i_rows, len_t), x0, lb, ub);

opt_info.resnorm = resnorm;
opt_info.residual = residual;
opt_info.exitflag = exitflag;
opt_info.output = output;

% calculate outputs
[~,out] = SolveVaneMountingBias(xopt, vel_n.Data, vel_e.Data, vel_d.Data, ...
    aoa_meas.Data, slip_meas.Data, airspeed.Data, Hb2i_rows, len_t);

% mean / std of ground speed errors
gsp_err = [vel_n.Data; vel_e.Data; vel_d.Data] - [out(:,1); out(:,2); out(:,3)];
mean_gsp_err = mean(gsp_err);
std_gsp_err = std(gsp_err);

% reconstruct wind velocity with mounting biases included
vel_wind = [vel_n.Data, vel_e.Data, vel_d.Data] - out(:,4:6);

%% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / 
% Output function / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

figure('color','w','name','Airflow Vane Mounting Bias Solution');

lines_ = lines(7);
plot_opacity = 0.5;

result_plots(1) = subplot(3,1,1); hold on; grid on; box on;
plot(time_resampled, rad2deg(aoa_meas.Data) - xopt(1), 'color', lines_(1,:));
plot(time_resampled, rad2deg(slip_meas.Data) - xopt(2), 'color', lines_(2,:));
legend('\alpha + b_\alpha', '\beta + b_\beta');
ylabel('Airflow Angles [deg]');

result_plots(2) = subplot(3,1,2); hold on; grid on; box on;
plot(time_resampled, vel_wind(:,1), '-.', 'color', lines_(1,:) * plot_opacity + ones(1,3) * (1 - plot_opacity));
plot(time_resampled, vel_wind(:,2), '-.', 'color', lines_(2,:) * plot_opacity + ones(1,3) * (1 - plot_opacity));
plot(time_resampled, vel_wind(:,3), '-.', 'color', lines_(3,:) * plot_opacity + ones(1,3) * (1 - plot_opacity));
plot(time_resampled([1 end]), xopt(3) * ones(1,2), 'color', lines_(1,:));
plot(time_resampled([1 end]), xopt(4) * ones(1,2), 'color', lines_(2,:));
plot(time_resampled([1 end]), zeros(1,2), 'linewidth', 2, 'color', lines_(3,:));
legend('w_n recnstr.', 'w_e recnstr.', 'w_d recnstr.', 'w_n fit', 'w_e fit', 'w_d = ZERO');
ylabel('Wind Components [m/s]');

result_plots(3) = subplot(3,1,3); hold on; grid on; box on;
plot(time_resampled, sqrt(vel_n.Data.^2 + vel_e.Data.^2 + vel_d.Data.^2), '-.', 'color', lines_(1,:) * plot_opacity + ones(1,3) * (1 - plot_opacity));
plot(time_resampled, sqrt(vel_wind(:,1).^2+vel_wind(:,2).^2), '-.', 'color', lines_(2,:) * plot_opacity + ones(1,3) * (1 - plot_opacity));
plot(time_resampled, airspeed.Data, '-.', 'color', lines_(3,:) * plot_opacity + ones(1,3) * (1 - plot_opacity));
plot(time_resampled, sqrt(out(:,1).^2 + out(:,2).^2 + out(:,3).^2), 'color', lines_(1,:));
plot(time_resampled([1 end]), norm(xopt(3) + xopt(4)) * ones(1,2), 'color', lines_(2,:));
ylabel('Speeds [m/s]');
legend('3D gnd. sp.', '2D wind sp.', 'u (x-body) airsp.');

xlabel('Time [s]');
linkaxes(result_plots(:),'x');
xlim(result_plots(:), time_resampled([1 end]));

% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / 
% Output function / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
function [f, out] = SolveVaneMountingBias(x, vel_n_data, vel_e_data, vel_d_data, ...
    aoa_meas_data, slip_meas_data, airspeed_data, Hb2i_rows, len_t)

% current solution
b_aoa = x(1);
b_slip = x(2);
wn = x(3);
we = x(4);

% reconstruct airspeed vector in inertial frame
vb_u = [ones(len_t,1), tan(slip_meas_data - deg2rad(b_slip)), tan(aoa_meas_data - deg2rad(b_aoa))];
vel_airsp = airspeed_data .* [ ...
    sum(Hb2i_rows(:,1:3) .* vb_u, 2), ...
    sum(Hb2i_rows(:,4:6) .* vb_u, 2), ...
    sum(Hb2i_rows(:,7:9) .* vb_u, 2)];

% reconstruct inertial velocity with zero down wind assumption
vel_n = vel_airsp(:,1) + wn;
vel_e = vel_airsp(:,2) + we;
vel_d = vel_airsp(:,3);
norm_vel_err = sqrt((vel_n_data - vel_n).^2 + (vel_e_data - vel_e).^2  + (vel_d_data - vel_d).^2);

% objective
f = [norm_vel_err; slip_meas_data - deg2rad(b_slip)];

% outputs
out = [vel_n, vel_e, vel_d, vel_airsp];