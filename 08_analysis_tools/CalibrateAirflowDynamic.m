% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
% Calibrate Airspeed Scale Factor
% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

% !!-- This script must be run *after running the logconv.m script. --!!

% A nonlinear least squares regression is solved for constant wind vector
% and airspeed scale factor by minimizing ground speed output errors.

% NOTE: this script is segmented into cells which should be run
%       sequentially after modifying settings at the top of each section

close all
clc

%% General Optimization Configuration

% start and end times of the data used
config.t_st_cal = 560;
config.t_ed_cal = 2380;

% load tube diameter and length from logs (implies these parameters have
% been previously measured and set to the airframe for the given flight)
config.tube_params_from_logs = true;

% If true the logged airflow angles are used, if false the airflow angles
% are calculated from the raw hall measurements
config.use_airflow_measurement = false;

% If the airflow angles are computed from the raw hall measurements this
% indicates if the logged params are used (true) or the manual inputs are
% used (false)
config.use_cal_av_params = false;

% Weigh the individual samples by a deviation from one g by the z
% acceleration in the inertial frame, else all samples have equal weight
config.use_accz_weighting = true;

% If true loiters are automatically detected and an individual bias and
% scale parameter is estimated for each detected loiter
config.auto_loiter_detections = false;

% If true activates additional outputs and plots
config.verbose = false;

% Different function definitions for the aoa and slip bias
% 0: b_aoa = P0 + P1 * (1 + g_load)
%    b_slip = A * tanh(B * (roll - C)) + D
% 1: b_aoa = P0 + P1 * aoa + P2 * (aoa + P3) * (airspeed + P4) + P5 * throttle
%    b_slip = P0 + P1 * (P2 * airspeed - P3) * (1 + tanh(P4 * (aoa - P5)) + P6 * slip
% 2: b_aoa = P0 + P1 * aoa + P2 * (aoa + P3) * (airspeed + P4) + P5 * throttle
%    b_slip = P0 + P1 * (P2 * airspeed - P3) * (1 + tanh(P4 * (aoa - P5)) + P6 * slip + P7 * tanh(P8 * (roll - P9));
config.calibration_function = 2;

% If true the throttle value might be used in any calibration function,
% else the respective param is force to being 0
config.calibration_use_throttle = false;

% Allow a dynamic airspeed scale
% False: scale = P0
% True: scale = P0 + P1 * gyro_z + P2 * airspeed
config.airspeed_scale_dynamic = true;

% If true the airspeed is corrected by an offset depending on the airflow
% angles (determined from wind tunnel data):
% dv = (0.15 * angles_norm - 0.0044 * angles_norm .^2)
config.airspeed_angle_correction = false;


config.magnetic_declination_deg = 0.0;

%% Pitot Tube Configuration
% select airframe / pitot configuration (see AirframePitotConfig.m):
% - 'manual-input'
% - 'techpod_long-probe_pre-05-2019'
% - 'ezg3_drotek'
% - 'techpod-agrofly_drotek'
config.airframe_pitot_config = 'manual-input';

config.radius_profile_cm = 1;      % radius of profile on which the probe is mounted (sphere or cylinder) [cm]
config.r_probe_tip_cm = 100;       % length of radial vector from center of profile to probe tip [cm]
config.theta_probe_tip_deg = 0;    % angle between horizon and radial vector [deg]
config.tube_dia = 1.5/1000;        % tube diameter [m] !!-this param is overwritten if loading tube params from logs
config.tube_len = 0.5;             % tube length [m]   !!-this param is overwritten if loading tube params from logs
config.pitot_type = 1;             % pitot type (drotek pitot = 0; custom pitot = 1)
config.mount_location = 1;         % 0 = wing (2D cylinder assumption), 1 = nose (3D sphere assumption)

%% Airflow Angles Config
config.cal_hall_aoa_rev = 1;
config.cal_hall_aoa_p0 = -143564503;
config.cal_hall_aoa_p1 = -29512516;
config.cal_hall_aoa_p2 = -31953;
config.cal_hall_aoa_p3 = -27419;
config.cal_hall_aoa_id = 50;
config.cal_hall_slip_rev = 1;
config.cal_hall_slip_p0 = 36081038;
config.cal_hall_slip_p1 = 29965695;
config.cal_hall_slip_p2 = 141878;
config.cal_hall_slip_p3 = 19035;
config.cal_hall_slip_id = 48;

% EZG3 Config 2
% config.cal_hall_aoa_rev = 1;
% config.cal_hall_aoa_p0 = 106825944;
% config.cal_hall_aoa_p1 = -27144586;
% config.cal_hall_aoa_p2 = -45294;
% config.cal_hall_aoa_p3 = 1557;
% config.cal_hall_aoa_id = 50;
% config.cal_hall_slip_rev = 1;
% config.cal_hall_slip_p0 = 36081038;
% config.cal_hall_slip_p1 = 29965695;
% config.cal_hall_slip_p2 = 141878;
% config.cal_hall_slip_p3 = 19035;
% config.cal_hall_slip_id = 48;

% EZG5
% config.cal_hall_aoa_rev = 1;
% config.cal_hall_aoa_p0 = 42662393;
% config.cal_hall_aoa_p1 = 51695989;
% config.cal_hall_aoa_p2 = 76009;
% config.cal_hall_aoa_p3 = 117747;
% config.cal_hall_aoa_id = 50;
% config.cal_hall_slip_rev = 1;
% config.cal_hall_slip_p0 = 67376245;
% config.cal_hall_slip_p1 = 33273923;
% config.cal_hall_slip_p2 = 55112;
% config.cal_hall_slip_p3 = 28187;
% config.cal_hall_slip_id = 48;

% EZG6
% config.cal_hall_aoa_rev = 1;
% config.cal_hall_aoa_p0 = 62943722;
% config.cal_hall_aoa_p1 = -29432471;
% config.cal_hall_aoa_p2 = 57698;
% config.cal_hall_aoa_p3 = -10655;
% config.cal_hall_aoa_id = 49;
% config.cal_hall_slip_rev = 1;
% config.cal_hall_slip_p0 = 48716976;
% config.cal_hall_slip_p1 = 33793644;
% config.cal_hall_slip_p2 = 26796;
% config.cal_hall_slip_p3 = 13161;
% config.cal_hall_slip_id = 50;

config.use_airflow_angles = true; % must be true for this script

%% Optimization Config
config.force_zero_wd = true;            % if true wd is optimized to equal zero instead of a nonzero constant value
config.aoa_offset_x = 0.165;            % aoa vane x offset with respect to the body frame [m]
config.aoa_offset_y = 0.315;            % aoa vane y offset with respect to the body frame [m]
config.slip_offset_x = 0.165;           % slip vane x offset with respect to the body frame [m]
config.slip_offset_z = -0.025;           % slip vane z offset with respect to the body frame [m]
config.airspeed_offset_y = -0.25;       % airspeed sensor y offset with respect to the body frame [m]
config.airspeed_offset_z = -0.03;        % airspeed sensor z offset with respect to the body frame [m]
config.t_movmean = 2;                   % time window for the movmean filter for the imu data [s]
config.weighting_sigma = 0.05;
config.segment_length = 50;
config.global_search = false;
config.global_search_num_points = 50;

%% Loiter Detection Config
config.window_size = 10;                % Window size to check if the segment is a circle [s]
config.overlap = 0.8;                   % Overlap between the windows [fraction]
config.min_radius = 15;                 % Min allowed loiter radius [m]                 
config.max_radius = 150;                % Max allowed loiter radius [m]
config.max_circle_fit_error = 15;       % Max allowed circle fit error [m]
config.min_airspeed = 8;                % Min airspeed to consider it a valid loiter [m/s]
config.max_airspeed_std = 0.75;         % Max airspeed std at the start of a loiter [m^2/s^2]
config.max_altitude_std = 2.0;          % Max airspeed std at the start of a loiter [m^2/s^2]
config.max_airspeed_change = 1.0;       % Max allowed airspeed change between a loiter [m/s]
config.max_radius_change = 0.1;         % Max allowed fraction change of the loiter radius
config.only_use_auto_loiter = true;     % Only use the loiters in auto mode, requires commander state being logged
config.n_loiters_required = 1.0;        % Number of loiters required, can be a fraction

%% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
% Plot wind reconstructions from (uncalibrated) true airspeed and ground
% speed measurements to approximate initial guess of NE wind components and
% choose a start and end time of appropriate calibration data. Calibration
% data should satisfy the following criteria:
% - in air at nominal airspeed
% - constant altitude
% - loitering with minimal bank for at least 3 cycles (hint: use stabilized
%   modes for collecting this data)
% - wind speeds are assumed to be constant within the seleted data

% ! START do not modify ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
% copy over the start and end time such that the script will not fail if
% the airflow measurements are not logged
config.st_time = config.t_st_cal;
config.end_time = config.t_ed_cal;

clc;
if (topics.airspeed.logged && topics.vehicle_local_position.logged && topics.vehicle_attitude.logged)
    [mean_raw_wind, tspan] = WindPlotsRaw(sysvector, topics, paramvector, params, config);
    disp(['Mean wind east = ', num2str(mean_raw_wind(1)), ' m/s']);
    disp(['Mean wind north = ', num2str(mean_raw_wind(2)), ' m/s']);
    disp(['Mean wind down = ', num2str(mean_raw_wind(3)), ' m/s']);
else
    disp('ERROR: logged topics are not sufficient for airspeed calibration.');
end
% ! END do not modify ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !

%% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
if ~strcmp(config.airframe_pitot_config, 'manual-input')
    % Load the predefined airframe params
    AirframePitotConfig;
end

% ! START do not modify ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
% load tube len/dia from params
if (config.tube_params_from_logs)
    if (params.cal_air_cmodel.logged && params.cal_air_tubelen.logged && params.cal_air_tubed_mm.logged)
        config.tube_dia = paramvector.cal_air_tubed_mm.Data(1)/1000;
        config.tube_len = paramvector.cal_air_tubelen.Data(1);
        if paramvector.cal_air_cmodel.Data(1) == 2
            config.pitot_type = 1;
        else
            config.pitot_type = 0;
        end
    else
        disp('ERROR: logged params are not sufficient for airspeed calibration.');    
    end
end

% scale factor from potential flow theory
if (config.mount_location == 0)
    % potential flow scales (cylinder, 2D)
    sf_theory = 1 / ( ...
        (1 - config.radius_profile_cm^2 / config.r_probe_tip_cm^2) * cosd(config.theta_probe_tip_deg)^2 + ...
        (1 + config.radius_profile_cm^2 / config.r_probe_tip_cm^2) * sind(config.theta_probe_tip_deg)^2 );
elseif (config.mount_location == 1)
% potential flow scales (sphere, 3D)
    sf_theory = 1 / ( ...
        (1 - config.radius_profile_cm^3 / config.r_probe_tip_cm^3) * cosd(config.theta_probe_tip_deg)^2 + ...
        (1 + config.radius_profile_cm^3 / config.r_probe_tip_cm^3 / 2) * sind(config.theta_probe_tip_deg)^2 );
else
    disp('ERROR: not a valid mounting location');
end
% ! END do not modify ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !


%% Detect the Loiters
if config.auto_loiter_detections
    if config.only_use_auto_loiter
        if topics.commander_state.logged
            commander_state = sysvector.commander_state_0.main_state;
        else
            error('only_use_auto_loiter requires the commander state being logged')
        end
        
    else
        commander_state = dummyvar(1);
    end
    
    [t_starts, t_ends] = DetectLoiters(sysvector.vehicle_local_position_0, sysvector.airspeed_0.true_airspeed_m_s, commander_state, config);
    
    if config.verbose
        for i=1:length(t_starts)
            config.t_st_cal = t_starts(i);
            config.t_ed_cal = t_ends(i);
            [mean_raw_wind, tspan] = WindPlotsRaw(sysvector, topics, paramvector, params, config);
            disp(['Mean wind east = ', num2str(mean_raw_wind(1)), ' m/s']);
            disp(['Mean wind north = ', num2str(mean_raw_wind(2)), ' m/s']);
            disp(['Mean wind down = ', num2str(mean_raw_wind(3)), ' m/s']);
        end
    end
else
    times = config.t_st_cal:config.segment_length:config.t_ed_cal;
    if times(end) < config.t_ed_cal
        times(end+1) = config.t_ed_cal;
    end
    
    t_starts = times(1:end-1);
    t_ends = times(2:end);
end

%% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
% Solve nonlinear least squares regression for sensor calibration
% (these settings may be adjusted, but typically do not need to be)

% set the start and end time
config.t_starts = t_starts;
config.t_ends = t_ends;

% initial guesses
sf0 = 1;                    % scale factor
wn0 = mean_raw_wind(1);     % wind speed north [m/s]
we0 = mean_raw_wind(2);     % wind speed east [m/s]
wd0 = mean_raw_wind(3);     % wind speed down [m/s]

if config.airspeed_scale_dynamic
    init_scale = [1; 0; 0];
    lb_scale = [0.1; -30; -30];
    ub_scale = [2.0;  30;  30];
else
    init_scale = [1];
    lb_scale = [0.1];
    ub_scale = [2.0];
end

if config.calibration_function == 0
    init_params = [0; 0; 0; 1; 0; 0];
    lb_params = [-0.2; -30; -30; -30; -30; -0.2];
    ub_params = [ 0.2;  30;  30;  30;  30;  0.2];
elseif config.calibration_function == 1
    init_params = [0; 0; 0; 0; 0; 0; 0; 1; 0; 0; 1; 0; 0];
    lb_params = [-0.2; -30; -30; -30; -30; -30; -0.2; -30; -30; -30; -30; -30; -30];
    ub_params = [ 0.2;  30;  30;  30;  30;  30;  0.2;  30;  30;  30;  30;  30;  30];
elseif config.calibration_function == 2
    init_params = [0; 0; 0; 0; 0; 0; 0; 1; 0; 0; 1; 0; 0; 0; 1; 0];
    lb_params = [-0.2; -30; -30; -30; -30; -30; -0.2; -30; -30; -30; -50; -30; -30; -30; -30; -30];
    ub_params = [ 0.2;  30;  30;  30;  30;  30;  0.2;  30;  30;  30;  50;  30;  30;  30;  30;  30];
else
    error('Unknown calibration function')
end

if config.force_zero_wd
    x0 = [init_scale; init_params; wn0 * ones(length(config.t_starts), 1); we0 * ones(length(config.t_starts), 1)];

    % bounds
    lb = [lb_scale; lb_params; -20 * ones(length(config.t_starts), 1); -20 * ones(length(config.t_starts), 1)];
    ub = [ub_scale; ub_params;  20 * ones(length(config.t_starts), 1);  20 * ones(length(config.t_starts), 1)];
else
    x0 = [init_scale; init_params; wn0 * ones(length(config.t_starts), 1); we0 * ones(length(config.t_starts), 1); wd0 * ones(length(config.t_starts), 1)];

    % bounds
    lb = [lb_scale; lb_params; -20 * ones(length(config.t_starts), 1); -20 * ones(length(config.t_starts), 1); -5 * ones(length(config.t_starts), 1)];
    ub = [ub_scale; ub_params;  20 * ones(length(config.t_starts), 1);  20 * ones(length(config.t_starts), 1);  5 * ones(length(config.t_starts), 1)];
end

disp(['wn initial = ',num2str(wn0),' m/s']);
disp(['we initial = ',num2str(we0),' m/s']);
disp(['wd initial = ',num2str(wd0),' m/s']);

% ! START do not modify ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
if (topics.differential_pressure.logged && topics.sensor_baro.logged && ...
        topics.vehicle_attitude.logged && topics.vehicle_local_position.logged && ...
        topics.airspeed.logged && topics.sensor_gyro.logged && ...
        ((topics.airflow_aoa.logged && topics.airflow_slip.logged) || topics.sensor_hall.logged))

    % solve
    [xopt, opt_info, mean_gsp_err, std_gsp_err, mean_wind_d, optimization_data] = ...
        SolveAirflowCalibrationDynamic(sysvector, topics, paramvector, params, x0, lb, ub, config);

    bounds_exceeded = xopt >= ub | xopt <= lb;
    if sum(bounds_exceeded)>0
        % bounds exceeded 
        disp(['WARNING: optimization bounds exceeded: [',int2str(bounds_exceeded'),']']);
    end
    
    disp(['Wn std: ', num2str(std(optimization_data.wn - optimization_data.wn_fit))])
    disp(['We std: ', num2str(std(optimization_data.we - optimization_data.we_fit))])
    disp(['Wd std: ', num2str(std(optimization_data.wd))])
    
    disp(['Wn std filtered: ', num2str(std(movmean(optimization_data.wn - optimization_data.wn_fit, 200)))])
    disp(['We std filtered: ', num2str(std(movmean(optimization_data.we - optimization_data.we_fit, 200)))])
    disp(['Wd std filtered: ', num2str(std(movmean(optimization_data.wd, 200)))])

    if config.airspeed_scale_dynamic
        disp(['SF P0 = ',num2str(xopt(1))]);
        disp(['SF sfPaspd = ',num2str(xopt(2))]);
        disp(['SF sfPgyrz = ',num2str(xopt(3))]);
        start_idx = 4;
    else
        disp(['SF = ',num2str(xopt(1)),'; SF(theory) = ',num2str(sf_theory)]);
        start_idx = 2;
    end
    
    if config.calibration_function == 0
        disp(['aoa bias (State Estimate Frame), P0 = ',num2str(xopt(start_idx+0))]);
        disp(['aoa bias (State Estimate Frame), P1 = ',num2str(xopt(start_idx+1))]);
        disp(['slip bias (State Estimate Frame), A = ',num2str(xopt(start_idx+2))]);
        disp(['slip bias (State Estimate Frame), B = ',num2str(xopt(start_idx+3))]);
        disp(['slip bias (State Estimate Frame), C = ',num2str(xopt(start_idx+4))]);
        disp(['slip bias (State Estimate Frame), D = ',num2str(xopt(start_idx+5))]);

        disp('Bias param array:')
        disp(num2str(xopt(start_idx:start_idx+5)', '% -8.4f'))

        if (params.sens_board_y_off.logged)
            disp(['aoa bias (IMU Frame), P0 = ',num2str(xopt(start_idx+0) - deg2rad(paramvector.sens_board_y_off.Data(1)))]);
        end
        if (params.sens_board_z_off.logged)
            disp(['slip bias (IMU Frame), D = ',num2str(xopt(start_idx+5) - deg2rad(paramvector.sens_board_z_off.Data(1)))]);
        end
    elseif config.calibration_function == 1
        disp(['aoa bias (State Estimate Frame), P0 = ',num2str(xopt(start_idx+0))]);
        disp(['aoa bias (State Estimate Frame), P1 = ',num2str(xopt(start_idx+1))]);
        disp(['aoa bias (State Estimate Frame), P2 = ',num2str(xopt(start_idx+2))]);
        disp(['aoa bias (State Estimate Frame), P3 = ',num2str(xopt(start_idx+3))]);
        disp(['aoa bias (State Estimate Frame), P4 = ',num2str(xopt(start_idx+4))]);
        disp(['aoa bias (State Estimate Frame), P5 = ',num2str(xopt(start_idx+5))]);

        disp(['slip bias (State Estimate Frame), P0 = ',num2str(xopt(start_idx+6))]);
        disp(['slip bias (State Estimate Frame), P1 = ',num2str(xopt(start_idx+7))]);
        disp(['slip bias (State Estimate Frame), P2 = ',num2str(xopt(start_idx+8))]);
        disp(['slip bias (State Estimate Frame), P3 = ',num2str(xopt(start_idx+9))]);
        disp(['slip bias (State Estimate Frame), P4 = ',num2str(xopt(start_idx+10))]);
        disp(['slip bias (State Estimate Frame), P5 = ',num2str(xopt(start_idx+11))]);
        disp(['slip bias (State Estimate Frame), P6 = ',num2str(xopt(start_idx+12))]);

        disp('Bias param array:')
        disp(num2str(xopt(start_idx:start_idx+12)', '% -8.4f'))

        if (params.sens_board_y_off.logged)
            disp(['aoa bias (IMU Frame), P0 = ',num2str(xopt(start_idx+0) - deg2rad(paramvector.sens_board_y_off.Data(1)))]);
        end
        if (params.sens_board_z_off.logged)
            disp(['slip bias (IMU Frame), P0 = ',num2str(xopt(start_idx+6) - deg2rad(paramvector.sens_board_z_off.Data(1)))]);
        end
    elseif config.calibration_function == 2
        disp(['aoa bias (State Estimate Frame), P0 = ',num2str(xopt(start_idx+0))]);
        disp(['aoa bias (State Estimate Frame), P1 = ',num2str(xopt(start_idx+1))]);
        disp(['aoa bias (State Estimate Frame), P2 = ',num2str(xopt(start_idx+2))]);
        disp(['aoa bias (State Estimate Frame), P3 = ',num2str(xopt(start_idx+3))]);
        disp(['aoa bias (State Estimate Frame), P4 = ',num2str(xopt(start_idx+4))]);
        disp(['aoa bias (State Estimate Frame), P5 = ',num2str(xopt(start_idx+5))]);

        disp(['slip bias (State Estimate Frame), P0 = ',num2str(xopt(start_idx+6))]);
        disp(['slip bias (State Estimate Frame), P1 = ',num2str(xopt(start_idx+7))]);
        disp(['slip bias (State Estimate Frame), P2 = ',num2str(xopt(start_idx+8))]);
        disp(['slip bias (State Estimate Frame), P3 = ',num2str(xopt(start_idx+9))]);
        disp(['slip bias (State Estimate Frame), P4 = ',num2str(xopt(start_idx+10))]);
        disp(['slip bias (State Estimate Frame), P5 = ',num2str(xopt(start_idx+11))]);
        disp(['slip bias (State Estimate Frame), P6 = ',num2str(xopt(start_idx+12))]);
        disp(['slip bias (State Estimate Frame), P7 = ',num2str(xopt(start_idx+13))]);
        disp(['slip bias (State Estimate Frame), P8 = ',num2str(xopt(start_idx+14))]);
        disp(['slip bias (State Estimate Frame), P9 = ',num2str(xopt(start_idx+15))]);
        
        disp('Bias param array:')
        disp(num2str(xopt(start_idx:start_idx+15)', '% -8.4f'))

        if (params.sens_board_y_off.logged)
            disp(['aoa bias (IMU Frame), P0 = ',num2str(xopt(start_idx+0) - deg2rad(paramvector.sens_board_y_off.Data(1)))]);
        end
        if (params.sens_board_z_off.logged)
            disp(['slip bias (IMU Frame), P0 = ',num2str(xopt(start_idx+6) - deg2rad(paramvector.sens_board_z_off.Data(1)))]);
        end
    end

    disp(['Mean of gsp. errors: ',num2str(mean_gsp_err),' m/s']);
    disp(['Standard deviation of gsp. errors: ',num2str(std_gsp_err),' m/s']);
    disp(['Average wind down: ',num2str(mean_wind_d),' m/s']);
    disp(['Set CAL_AIR_SCALE = ',num2str(xopt(1))]);
else
    error('ERROR: logged topics are not sufficient for airspeed calibration.');
end
% ! END do not modify ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !

%% Error plots
plot_bias = false;
if config.calibration_function == 0
    x_slip = linspace(-40, 40);
    x_aoa = linspace(-2, -0.5);
    xlabel_aoa = 'g-loading [-]';
    xlabel_slip = 'roll [deg]';
    aoa_bias = rad2deg(xopt(start_idx+0) + xopt(start_idx+1) * x_aoa);
    slip_bias = rad2deg(xopt(start_idx+2) * tanh(xopt(start_idx+3) * deg2rad(x_slip)) + xopt(start_idx+4));
    plot_bias = true;
end

if (plot_bias)
    figure('color','w');
    title('Bias Function')
    bias_plots(1) = subplot(2,1,1); hold on; grid on; box on;
    plot(x_aoa, aoa_bias)
    xlabel(xlabel_aoa)
    ylabel('aoa bias [deg]')

    bias_plots(2) = subplot(2,1,2); hold on; grid on; box on;
    plot(x_slip, slip_bias)
    xlabel(xlabel_slip)
    ylabel('slip bias [deg]')
end

wn_filt = movmean(optimization_data.wn, 200);
we_filt = movmean(optimization_data.we, 200);
wd_filt = movmean(optimization_data.wd, 200);

figure('color','w');
title('After optimization')
validation_plots(1) = subplot(3,2,1); hold on; grid on; box on;
plot(optimization_data.g_load, wn_filt, 'o')
plot(optimization_data.g_load, we_filt, 'o')
plot(optimization_data.g_load, wd_filt, 'o')
legend('w_n', 'w_e', 'w_d')
xlabel('g-loading [-]')
ylabel('wind estimate [m/s]')

validation_plots(end+1) = subplot(3,2,2); hold on; grid on; box on;
plot(optimization_data.airspeed, wn_filt, 'o')
plot(optimization_data.airspeed, we_filt, 'o')
plot(optimization_data.airspeed, wd_filt, 'o')
legend('w_n', 'w_e', 'w_d')
xlabel('airspeed [m/s]')
ylabel('wind estimate [m/s]')

validation_plots(end+1) = subplot(3,2,3); hold on; grid on; box on;
plot(rad2deg(optimization_data.roll), wn_filt, 'o')
plot(rad2deg(optimization_data.roll), we_filt, 'o')
plot(rad2deg(optimization_data.roll), wd_filt, 'o')
legend('w_n', 'w_e', 'w_d')
xlabel('roll [deg]')
ylabel('wind estimate [m/s]')

validation_plots(end+1) = subplot(3,2,4); hold on; grid on; box on;
plot(rad2deg(optimization_data.aoa), wn_filt, 'o')
plot(rad2deg(optimization_data.aoa), we_filt, 'o')
plot(rad2deg(optimization_data.aoa), wd_filt, 'o')
legend('w_n', 'w_e', 'w_d')
xlabel('aoa [deg]')
ylabel('wind estimate [m/s]')

validation_plots(end+1) = subplot(3,2,5); hold on; grid on; box on;
plot(rad2deg(optimization_data.slip), wn_filt, 'o')
plot(rad2deg(optimization_data.slip), we_filt, 'o')
plot(rad2deg(optimization_data.slip), wd_filt, 'o')
legend('w_n', 'w_e', 'w_d')
xlabel('slip [deg]')
ylabel('wind estimate [m/s]')

validation_plots(end+1) = subplot(3,2,6); hold on; grid on; box on;
plot(optimization_data.throttle, wn_filt, 'o')
plot(optimization_data.throttle, we_filt, 'o')
plot(optimization_data.throttle, wd_filt, 'o')
legend('w_n', 'w_e', 'w_d')
xlabel('throttle [-]')
ylabel('wind estimate [m/s]')
