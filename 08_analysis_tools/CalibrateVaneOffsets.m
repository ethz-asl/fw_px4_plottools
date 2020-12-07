% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
% Calibrate Airflow Vane Mounting Offsets
% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

% !!-- This script must be run *after running the logconv.m script. --!!

% NOTE: airspeed scale, tube pressure drop, and vane position mapping
%       calibrations are assumed completed before running this script.

% Nonlinear least squares solution:
% min_x ||f(x,u)||
% subject to
% w_d = 0, mean slip = 0
% where 
% y = f(x,u)
% y_ref = [vI_n, vI_e, vI_d, slip]
% x = [b_aoa, b_slip, w_n, w_e]
% u = [u, aoa, slip, phi, theta, psi]

%% find vanes offset calibration maneuvers

% check channel 6 (aux2 - remote) for enabled sysid experiments
% XXX: probably should also check that we are in stablized.. and maybe have
% a logged state indicating when the sysid is running..

% find sysid mode on + calibration man selected

cal_sel_nr_ = 600;

sysid_switch_ = sysvector.input_rc_0.values_6;
sid_man_sel_ = resample(paramvector.sid_sel, sysid_switch_.Time, 'zoh');

rising_falling_edges = [0; diff(sysid_switch_.Data > 1600)]; % 1600 is arbitrary threshold

% set starts and ends of experiments via rising and falling edges
idx_exp_st = find(rising_falling_edges > 0);
idx_exp_ed = find(rising_falling_edges < 0);
idx_exp_all = [idx_exp_st(idx_exp_st<idx_exp_ed(end)), ...
    idx_exp_ed(idx_exp_ed>idx_exp_st(1)) sid_man_sel_.Data(idx_exp_st)];

idx_av_off_cal = idx_exp_all(idx_exp_all(:, end) == cal_sel_nr_, :)


bp = 0;

%% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
% Calibration data should satisfy the following criteria:
% - in air at nominal airspeed
% - constant altitude
% - executing "figure-8" with minimal bank for at least 2 cycles (hint: use
%   stabilized modes for collecting this data) -- note both left and right
%   turns are needed to make the assumption that average slip in nominal
%   conditions is = zero.
% - wind speeds are assumed to be constant within the seleted data
% - no vertical wind is present (assumed zero)
% NOTE: slip will be assumed = zero in nominal stabilized conditions, this
% may not be true in reality, but with wind, the mounting offset is
% otherwise unobservable.

% start and end times (modify these)
t_st_cal = -1;
t_ed_cal = 10000;

% if hall calibration params were not present in the logs, they
% must be defined here to execute the mounting offset calibration.
% !! default values given here are for two ASL vanes, shown as
%    example for typical calibration values - these will not be
%    valid for any different vane !!

% aoa
config.cal_hall_aoa_rev = timeseries(-1, 0);
config.cal_hall_aoa_p0 = timeseries(-14694678, 0);
config.cal_hall_aoa_p1 = timeseries(-21239312, 0);
config.cal_hall_aoa_p2 = timeseries(51980, 0);
config.cal_hall_aoa_p3 = timeseries(-2845, 0);
config.cal_hall_aoa_id = timeseries(0, 0);
% slip
config.cal_hall_slip_rev = timeseries(1, 0);
config.cal_hall_slip_p0 = timeseries(2696759, 0);
config.cal_hall_slip_p1 = timeseries(-25891024, 0);
config.cal_hall_slip_p2 = timeseries(53064, 0);
config.cal_hall_slip_p3 = timeseries(-7467, 0);
config.cal_hall_slip_id = timeseries(1, 0);
% general configs
config.use_cal_av_params = true;
config.use_airflow_measurement = true;

% Get the measured airflow angles
[aoa_meas, slip_meas, aoa_logged, slip_logged] = CalibrateAirflowAngles(sysvector, topics, paramvector, params, config);
sysvector.aoa_meas = aoa_meas;
sysvector.slip_meas = slip_meas;

clc;
if (topics.airspeed.logged && ...
        topics.vehicle_gps_position.logged && ...
        topics.vehicle_attitude.logged && ...
        aoa_logged && ...
        slip_logged)

    
    [mean_states, tspan] = VaneMountingCalibrationData(sysvector, topics, [t_st_cal, t_ed_cal]);
    disp(['Mean wind north = ', num2str(mean_states(1)), ' m/s']);
    disp(['Mean wind east = ', num2str(mean_states(2)), ' m/s']);
    disp(['Mean sideslip = ', num2str(mean_states(3)), ' deg']);
else
    disp('ERROR: logged topics/params are not sufficient for mounting offset calibration.');
end

%% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
% Solve nonlinear least squares regression for sensor calibration

% initial guesses
b_aoa0 = 0;                 % aoa mounting bias [deg]
b_slip0 = mean_states(3);	% slip mounting bias [deg]
wn0 = mean_states(1);       % wind speed north [m/s]
we0 = mean_states(2);       % wind speed east [m/s]
x0 = [b_aoa0; b_slip0; wn0; we0];

% bounds
lb = [-20; -20; -20; -20];
ub = [20; 20; 20; 20];

clc;
if (topics.airspeed.logged && ...
        topics.vehicle_gps_position.logged && ...
        topics.vehicle_attitude.logged && ...
        aoa_logged && ...
        slip_logged)

    % solve
    [xopt, opt_info, mean_gsp_err, std_gsp_err] = ...
        SolveVaneOffsets(sysvector, topics, tspan, x0, lb, ub);

    disp(['b_aoa = ',num2str(xopt(1)),' deg']);
    disp(['b_slip = ',num2str(xopt(2)),' deg']);
    disp(['wn = ',num2str(xopt(3)),' m/s']);
    disp(['we = ',num2str(xopt(4)),' m/s']);
    bounds_exceeded = xopt >= ub | xopt <= lb;
    if sum(bounds_exceeded)>0
        % bounds exceeded 
        disp(['WARNING: optimization bounds exceeded: [',int2str(bounds_exceeded'),']']);
    end
    disp(['Mean of gsp. vel. errors: ',num2str(mean_gsp_err),' m/s']);
    disp(['Standard deviation of gsp. vel. errors: ',num2str(std_gsp_err),' m/s']);
    disp(['Set CAL_AOA_OFF = ',num2str(xopt(1))]);
    disp(['Set CAL_SLIP_OFF = ',num2str(xopt(2))]);
else
    disp('ERROR: logged topics/params are not sufficient for mounting offset calibration.');
end