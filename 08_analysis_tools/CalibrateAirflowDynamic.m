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
config.t_st_cal = 0;
config.t_ed_cal = 10000;

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
config.auto_loiter_detections = true;

% If true activates additional outputs and plots
config.verbose = false;

% Different function definitions for the aoa and slip bias
config.calibration_function = 0;

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
config.aoa_offset_x = 0.170;            % aoa vane x offset with respect to the body frame [m]
config.aoa_offset_y = 0.225;            % aoa vane y offset with respect to the body frame [m]
config.slip_offset_x = 0.170;           % slip vane x offset with respect to the body frame [m]
config.slip_offset_z = -0.03;           % slip vane z offset with respect to the body frame [m]
config.airspeed_offset_y = -0.25;       % airspeed sensor y offset with respect to the body frame [m]
config.airspeed_offset_z = 0.05;        % airspeed sensor z offset with respect to the body frame [m]
config.t_movmean = 2;                   % time window for the movmean filter for the imu data [s]
config.weighting_sigma = 0.05;
config.segment_length = 100;

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

if config.calibration_function == 0
    init_params = [0; 0; 0; 1; 0];
    lb_params = [-0.2; -30; -30; -30; -0.2];
    ub_params = [ 0.2;  30;  30;  30;  0.2];
elseif config.calibration_function == 1
    init_params = [0; 0; 0; 0; 0; 0];
    lb_params = [-0.2; -30; -30; -0.2; -30; -30];
    ub_params = [ 0.2;  30;  30;  0.2;  30;  30];
elseif config.calibration_function == 2
    init_params = [0; 0; 0; 0; 1; 0; 1; 0; 1; 0; 0; 1; 0; 1; 0; 1; 0; 1];
    lb_params = [-0.2; -30; -30; -30; -30; -30; -30; -30; -30; -0.2; -30; -30; -30; -30; -30; -30; -30; -30];
    ub_params = [ 0.2;  30;  30;  30;  30;  30;  30;  30;  0.2;  30;  30;  30;  30;  30;  30;  30;  30;  30];
elseif config.calibration_function == 3
    init_params = [0; 0; 0; 0; 0; 1; 0; 0];
    lb_params = [-30; -30; -0.2; -30; -30; -30; -0.2; -30];
    ub_params = [ 30;  30;  0.2;  30;  30;  30;  0.2;  30];
elseif config.calibration_function == 4
    init_params = [0; 0; 1; 0; 0; 0; 0; 1; 0; 0];
    lb_params = [-30; -30; -30; -30; -0.2; -30; -30; -30; -0.2; -30];
    ub_params = [ 30;  30;  30;  30;  0.2;  30;  30;  30;  0.2;  30];
else
    error('Unknown calibration function')
end
if config.force_zero_wd
    x0 = [sf0; init_params; wn0 * ones(length(config.t_starts), 1); we0 * ones(length(config.t_starts), 1)];

    % bounds
    lb = [0.1; lb_params; -20 * ones(length(config.t_starts), 1); -20 * ones(length(config.t_starts), 1)];
    ub = [  2; ub_params;  20 * ones(length(config.t_starts), 1);  20 * ones(length(config.t_starts), 1)];
else
    x0 = [sf0; init_params; wn0 * ones(length(config.t_starts), 1); we0 * ones(length(config.t_starts), 1); wd0 * ones(length(config.t_starts), 1)];

    % bounds
    lb = [0.1; lb_params; -20 * ones(length(config.t_starts), 1); -20 * ones(length(config.t_starts), 1); -5 * ones(length(config.t_starts), 1)];
    ub = [  2; ub_params;  20 * ones(length(config.t_starts), 1);  20 * ones(length(config.t_starts), 1);  5 * ones(length(config.t_starts), 1)];
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

    disp(['SF = ',num2str(xopt(1)),'; SF(theory) = ',num2str(sf_theory)]);
    
    if config.calibration_function == 0
        disp(['aoa bias (State Estimate Frame), P0 = ',num2str(xopt(2))]);
        disp(['aoa bias (State Estimate Frame), P1 = ',num2str(xopt(3))]);
        disp(['slip bias (State Estimate Frame), A = ',num2str(xopt(4))]);
        disp(['slip bias (State Estimate Frame), B = ',num2str(xopt(5))]);
        disp(['slip bias (State Estimate Frame), C = ',num2str(xopt(6))]);
        if (params.sens_board_y_off.logged)
            disp(['aoa bias (IMU Frame), P0 = ',num2str(xopt(2) - deg2rad(paramvector.sens_board_y_off.Data(1)))]);
        end
        if (params.sens_board_z_off.logged)
            disp(['slip bias (IMU Frame), C = ',num2str(xopt(6) - deg2rad(paramvector.sens_board_z_off.Data(1)))]);
        end
    elseif config.calibration_function == 1
        disp(['aoa bias (State Estimate Frame), P0 = ',num2str(xopt(2))]);
        disp(['aoa bias (State Estimate Frame), P1 = ',num2str(xopt(3))]);
        disp(['aoa bias (State Estimate Frame), P2 = ',num2str(xopt(4))]);

        disp(['slip bias (State Estimate Frame), P0 = ',num2str(xopt(5))]);
        disp(['slip bias (State Estimate Frame), P1 = ',num2str(xopt(6))]);
        disp(['slip bias (State Estimate Frame), P2 = ',num2str(xopt(7))]);
        if (params.sens_board_y_off.logged)
            disp(['aoa bias (IMU Frame), P0 = ',num2str(xopt(2) - deg2rad(paramvector.sens_board_y_off.Data(1)))]);
        end
        if (params.sens_board_z_off.logged)
            disp(['slip bias (IMU Frame), P0 = ',num2str(xopt(5) - deg2rad(paramvector.sens_board_z_off.Data(1)))]);
        end
    elseif config.calibration_function == 2
        disp(['scale factor, sfPdp = ',num2str(xopt(2))]);
        disp(['scale factor, sfPgyrz = ',num2str(xopt(3))]);
        
        disp(['aoa bias (State Estimate Frame), P0 = ',num2str(xopt(4))]);
        disp(['aoa bias (State Estimate Frame), P1 = ',num2str(xopt(5))]);
        disp(['aoa bias (State Estimate Frame), P2 = ',num2str(xopt(6))]);
        disp(['aoa bias (State Estimate Frame), P3 = ',num2str(xopt(7))]);
        disp(['aoa bias (State Estimate Frame), P4 = ',num2str(xopt(8))]);
        disp(['aoa bias (State Estimate Frame), P5 = ',num2str(xopt(9))]);
        disp(['aoa bias (State Estimate Frame), P6 = ',num2str(xopt(10))]);

        disp(['slip bias (State Estimate Frame), P0 = ',num2str(xopt(11))]);
        disp(['slip bias (State Estimate Frame), P1 = ',num2str(xopt(12))]);
        disp(['slip bias (State Estimate Frame), P2 = ',num2str(xopt(13))]);
        disp(['slip bias (State Estimate Frame), P3 = ',num2str(xopt(14))]);
        disp(['slip bias (State Estimate Frame), P4 = ',num2str(xopt(15))]);
        disp(['slip bias (State Estimate Frame), P5 = ',num2str(xopt(16))]);
        disp(['slip bias (State Estimate Frame), P6 = ',num2str(xopt(17))]);
        disp(['slip bias (State Estimate Frame), P7 = ',num2str(xopt(18))]);
        disp(['slip bias (State Estimate Frame), P8 = ',num2str(xopt(19))]);
        if (params.sens_board_y_off.logged)
            disp(['aoa bias (IMU Frame), P0 = ',num2str(xopt(2) - deg2rad(paramvector.sens_board_y_off.Data(1)))]);
        end
        if (params.sens_board_z_off.logged)
            disp(['slip bias (IMU Frame), P0 = ',num2str(xopt(9) - deg2rad(paramvector.sens_board_z_off.Data(1)))]);
        end
    elseif config.calibration_function == 3
        disp(['scale factor, sfPgyrz = ',num2str(xopt(2))]);
        disp(['scale factor, sfPslip = ',num2str(xopt(3))]);
        
        disp(['aoa bias (State Estimate Frame), P0 = ',num2str(xopt(4))]);
        disp(['aoa bias (State Estimate Frame), P1 = ',num2str(xopt(5))]);

        disp(['slip bias (State Estimate Frame), A = ',num2str(xopt(6))]);
        disp(['slip bias (State Estimate Frame), B = ',num2str(xopt(7))]);
        disp(['slip bias (State Estimate Frame), C = ',num2str(xopt(8))]);
        disp(['slip bias (State Estimate Frame), D = ',num2str(xopt(9))]);
        if (params.sens_board_y_off.logged)
            disp(['aoa bias (IMU Frame), P0 = ',num2str(xopt(4) - deg2rad(paramvector.sens_board_y_off.Data(1)))]);
        end
        if (params.sens_board_z_off.logged)
            disp(['slip bias (IMU Frame), D = ',num2str(xopt(9) - deg2rad(paramvector.sens_board_z_off.Data(1)))]);
        end
    elseif config.calibration_function == 4
        disp(['scale factor, sfPgyrz = ',num2str(xopt(2))]);
        disp(['scale factor, sfAslip = ',num2str(xopt(3))]);
        disp(['scale factor, sfBslip = ',num2str(xopt(4))]);
        disp(['scale factor, sfCslip = ',num2str(xopt(5))]);
        
        disp(['aoa bias (State Estimate Frame), P0 = ',num2str(xopt(6))]);
        disp(['aoa bias (State Estimate Frame), P1 = ',num2str(xopt(7))]);

        disp(['slip bias (State Estimate Frame), A = ',num2str(xopt(8))]);
        disp(['slip bias (State Estimate Frame), B = ',num2str(xopt(9))]);
        disp(['slip bias (State Estimate Frame), C = ',num2str(xopt(10))]);
        disp(['slip bias (State Estimate Frame), D = ',num2str(xopt(11))]);
        if (params.sens_board_y_off.logged)
            disp(['aoa bias (IMU Frame), P0 = ',num2str(xopt(6) - deg2rad(paramvector.sens_board_y_off.Data(1)))]);
        end
        if (params.sens_board_z_off.logged)
            disp(['slip bias (IMU Frame), D = ',num2str(xopt(11) - deg2rad(paramvector.sens_board_z_off.Data(1)))]);
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
    aoa_bias = rad2deg(xopt(2) + xopt(3) * x_aoa);
    slip_bias = rad2deg(xopt(4) * tanh(xopt(5) * deg2rad(x_slip)) + xopt(6));
    plot_bias = true;
elseif config.calibration_function == 1
    x_slip = linspace(-24, 24);
    x_aoa = linspace(-24, 24);
    xlabel_aoa = 'aoa [deg]';
    xlabel_slip = 'slip [deg]';
    aoa_bias = rad2deg(xopt(2) + xopt(3) * deg2rad(x_aoa) + xopt(4) * deg2rad(x_aoa) .* deg2rad(x_aoa));
    slip_bias = rad2deg(xopt(5) + xopt(6) * deg2rad(x_slip) + xopt(7) * deg2rad(x_slip) .* deg2rad(x_slip));
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

figure('color','w');
title('After optimization')
validation_plots(1) = subplot(3,2,1); hold on; grid on; box on;
plot(optimization_data.g_load, optimization_data.wn, 'o')
plot(optimization_data.g_load, optimization_data.we, 'o')
plot(optimization_data.g_load, optimization_data.wd, 'o')
legend('w_n', 'w_e', 'w_d')
xlabel('g-loading [-]')
ylabel('wind estimate [m/s]')

validation_plots(end+1) = subplot(3,2,2); hold on; grid on; box on;
plot(optimization_data.airspeed, optimization_data.wn, 'o')
plot(optimization_data.airspeed, optimization_data.we, 'o')
plot(optimization_data.airspeed, optimization_data.wd, 'o')
legend('w_n', 'w_e', 'w_d')
xlabel('airspeed [m/s]')
ylabel('wind estimate [m/s]')

validation_plots(end+1) = subplot(3,2,3); hold on; grid on; box on;
plot(rad2deg(optimization_data.roll), optimization_data.wn, 'o')
plot(rad2deg(optimization_data.roll), optimization_data.we, 'o')
plot(rad2deg(optimization_data.roll), optimization_data.wd, 'o')
legend('w_n', 'w_e', 'w_d')
xlabel('roll [deg]')
ylabel('wind estimate [m/s]')

validation_plots(end+1) = subplot(3,2,4); hold on; grid on; box on;
plot(rad2deg(optimization_data.aoa), optimization_data.wn, 'o')
plot(rad2deg(optimization_data.aoa), optimization_data.we, 'o')
plot(rad2deg(optimization_data.aoa), optimization_data.wd, 'o')
legend('w_n', 'w_e', 'w_d')
xlabel('aoa [deg]')
ylabel('wind estimate [m/s]')

validation_plots(end+1) = subplot(3,2,5); hold on; grid on; box on;
plot(rad2deg(optimization_data.slip), optimization_data.wn, 'o')
plot(rad2deg(optimization_data.slip), optimization_data.we, 'o')
plot(rad2deg(optimization_data.slip), optimization_data.wd, 'o')
legend('w_n', 'w_e', 'w_d')
xlabel('slip [deg]')
ylabel('wind estimate [m/s]')

validation_plots(end+1) = subplot(3,2,6); hold on; grid on; box on;
plot(optimization_data.throttle, optimization_data.wn, 'o')
plot(optimization_data.throttle, optimization_data.we, 'o')
plot(optimization_data.throttle, optimization_data.wd, 'o')
legend('w_n', 'w_e', 'w_d')
xlabel('throttle [-]')
ylabel('wind estimate [m/s]')

%%
figure()
test(1) = subplot(2,1,1); hold on; grid on; box on;
plot(optimization_data.time, optimization_data.wd)

test(2) = subplot(2,1,2); hold on; grid on; box on;
plot(sysvector.vehicle_local_position_0.vz.Time, sysvector.vehicle_local_position_0.vz.Data)

linkaxes(test(:),'x');
