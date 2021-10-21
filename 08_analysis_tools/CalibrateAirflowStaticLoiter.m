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

% If true loiters are automatically detected and an individual bias and
% scale parameter is estimated for each detected loiter
config.auto_loiter_detections = true;

% If true activates additional outputs and plots
config.verbose = true;

% If true, the data between successive runs are stacked and used in fitting
% the biases
config.stack_data = false;
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

config.use_airflow_angles = true; % must be true for this script

%% Optimization Config
config.force_zero_wd = true;            % if true wd is optimized to equal zero instead of a nonzero constant value
config.pitching_calibration = false;    % aoa calibration based on pitching maneuvers
config.slip_bias_deg = 1.0;             % slip bias for the pitching maneuvers [deg]
config.aoa_offset_x = 0.185;            % aoa vane x offset with respect to the body frame [m]
config.aoa_offset_y = 0.455;            % aoa vane y offset with respect to the body frame [m]
config.slip_offset_x = 0.185;           % slip vane x offset with respect to the body frame [m]
config.slip_offset_z = -0.08;           % slip vane z offset with respect to the body frame [m]

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
if (topics.airspeed.logged && topics.vehicle_gps_position.logged && topics.vehicle_attitude.logged)
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
    t_starts = config.t_st_cal;
    t_ends = config.t_ed_cal;
end

%% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
% Solve nonlinear least squares regression for sensor calibration
% (these settings may be adjusted, but typically do not need to be)

optimization_data = struct();

for i=1:length(t_starts)
    % set the start and end time
    config.t_st_cal = t_starts(i);
    config.t_ed_cal = t_ends(i);
    
    % initial guesses
    sf0 = 1;                    % scale factor
    ba0 = 0;                    % bias alpha vane [rad]
    bs0 = 0;                    % bias slip vane [rad]
    wn0 = mean_raw_wind(1);     % wind speed north [m/s]
    we0 = mean_raw_wind(2);     % wind speed east [m/s]
    wd0 = mean_raw_wind(3);     % wind speed down [m/s]
    if config.pitching_calibration
        x0 = [sf0; ba0; wd0];

        % bounds
        lb = [0.1; -0.2; -5];
        ub = [  2;  0.2;  5];

    elseif config.force_zero_wd
        x0 = [wn0; we0; sf0; ba0; bs0];

        % bounds
        lb = [-20; -20; 0.1; -0.2; -0.2];
        ub = [ 20;  20;   2;  0.2;  0.2];
    else
        x0 = [wn0; we0; sf0; ba0; bs0; wd0];

        % bounds
        lb = [-20; -20; 0.1; -0.2; -0.2; -5];
        ub = [ 20;  20;   2;  0.2;  0.2;  5];
    end

    if length(t_starts) == 1
        disp(['wn initial = ',num2str(wn0),' m/s']);
        disp(['we initial = ',num2str(we0),' m/s']);
        disp(['wd initial = ',num2str(wd0),' m/s']);
    end

    % ! START do not modify ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
    if (topics.differential_pressure.logged && topics.sensor_baro.logged && ...
            topics.vehicle_attitude.logged && topics.vehicle_gps_position.logged && ...
            topics.airspeed.logged && topics.sensor_gyro.logged && ...
            ((topics.airflow_aoa.logged && topics.airflow_slip.logged) || topics.sensor_hall.logged))

        % solve
        [xopt, opt_info, mean_gsp_err, std_gsp_err, mean_wind_d, loiter_data] = ...
            SolveAirflowCalibration(sysvector, topics, paramvector, params, x0, lb, ub, config);
        
        fn = fieldnames(loiter_data);
        for k=1:numel(fn)
            if isfield(optimization_data, fn(k))
                optimization_data.(fn{k})(end+1) = loiter_data.(fn{k});
            else
                optimization_data.(fn{k}) = loiter_data.(fn{k});
            end
        end

        bounds_exceeded = xopt >= ub | xopt <= lb;
        if sum(bounds_exceeded)>0
            % bounds exceeded 
            disp(['WARNING: optimization bounds exceeded: [',int2str(bounds_exceeded'),']']);
        end
        
        % display the values only if it is for a single loiter
        if length(t_starts) == 1
            if config.pitching_calibration
                disp(['SF = ',num2str(xopt(1)),'; SF(theory) = ',num2str(sf_theory)]);
                disp(['wd = ',num2str(xopt(3)),' m/s']);
                disp(['aoa bias (State Estimate Frame) = ',num2str(rad2deg(xopt(2))),' deg']);
                if (params.sens_board_y_off.logged)
                    disp(['aoa bias (IMU Frame) = ',num2str(rad2deg(xopt(2)) - paramvector.sens_board_y_off.Data(1)),' deg']);
                end
                disp(['Mean of gsp. errors: ',num2str(mean_gsp_err),' m/s']);
                disp(['Standard deviation of gsp. errors: ',num2str(std_gsp_err),' m/s']);
                disp(['Set CAL_AIR_SCALE = ',num2str(xopt(1))]);

            else
                disp(['wn = ',num2str(xopt(1)),' m/s']);
                disp(['we = ',num2str(xopt(2)),' m/s']);
                if ~config.force_zero_wd
                    disp(['wd = ',num2str(xopt(6)),' m/s']);
                end
                disp(['SF = ',num2str(xopt(3)),'; SF(theory) = ',num2str(sf_theory)]);
                disp(['aoa bias (State Estimate Frame) = ',num2str(rad2deg(xopt(4))),' deg']);
                disp(['slip bias (State Estimate Frame) = ',num2str(rad2deg(xopt(5))),' deg']);
                if (params.sens_board_y_off.logged)
                    disp(['aoa bias (IMU Frame) = ',num2str(rad2deg(xopt(4)) - paramvector.sens_board_y_off.Data(1)),' deg']);
                end
                if (params.sens_board_z_off.logged)
                    disp(['slip bias (IMU Frame) = ',num2str(rad2deg(xopt(5)) - paramvector.sens_board_z_off.Data(1)),' deg']);
                end
                disp(['Mean of gsp. errors: ',num2str(mean_gsp_err),' m/s']);
                disp(['Standard deviation of gsp. errors: ',num2str(std_gsp_err),' m/s']);
                disp(['Average wind down: ',num2str(mean_wind_d),' m/s']);
                disp(['Set CAL_AIR_SCALE = ',num2str(xopt(3))]);    
            end
        end

    else
        error('ERROR: logged topics are not sufficient for airspeed calibration.');
    end
end
% ! END do not modify ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !

%% Stack data from previous runs if required
if config.stack_data
    if exist('previous_optimization_data','var')
        fn = fieldnames(previous_optimization_data);
        for k=1:numel(fn)
            if isfield(optimization_data, fn(k))
                optimization_data.(fn{k}) = [optimization_data.(fn{k}), previous_optimization_data.(fn{k})];
            else
                optimization_data.(fn{k}) = previous_optimization_data.(fn{k});
            end
        end
    end
    previous_optimization_data = optimization_data;
end

%% Display the optimization results for more than one loiter
if length(t_starts) > 1
    lines_ = lines(7);
    
    figure('color','w');
    title('Slip Angle Plots')
    slip_plots(1) = subplot(3,3,1); hold on; grid on; box on;
    plot(rad2deg(optimization_data.roll), rad2deg(optimization_data.slip_bias), 'o');
    ylabel('slip bias [deg]');
    xlabel('roll [deg]')
    
    slip_plots(end+1) = subplot(3,3,2); hold on; grid on; box on;
    plot(optimization_data.airspeed, rad2deg(optimization_data.slip_bias), 'o');
    ylabel('slip bias [deg]');
    xlabel('airspeed [m/s]')
    
    slip_plots(end+1) = subplot(3,3,3); hold on; grid on; box on;
    plot(optimization_data.g_load, rad2deg(optimization_data.slip_bias), 'o');
    ylabel('slip bias [deg]');
    xlabel('g-load [-]')
    
    slip_plots(end+1) = subplot(3,3,4); hold on; grid on; box on;
    plot(optimization_data.throttle, rad2deg(optimization_data.slip_bias), 'o');
    ylabel('slip bias [deg]');
    xlabel('throttle [-]')
    
    slip_plots(end+1) = subplot(3,3,5); hold on; grid on; box on;
    plot(rad2deg(optimization_data.roll), rad2deg(optimization_data.slip), 'o');
    ylabel('corrected slip [deg]');
    xlabel('roll [deg]')
    
    slip_plots(end+1) = subplot(3,3,6); hold on; grid on; box on;
    plot(optimization_data.airspeed, rad2deg(optimization_data.slip), 'o');
    ylabel('corrected slip [deg]');
    xlabel('airspeed [m/s]')
    
    slip_plots(end+1) = subplot(3,3,7); hold on; grid on; box on;
    plot(optimization_data.throttle, rad2deg(optimization_data.slip), 'o');
    ylabel('corrected slip [deg]');
    xlabel('throttle [m/s]')

    slip_plots(end+1) = subplot(3,3,8); hold on; grid on; box on;
    plot(rad2deg(optimization_data.roll), rad2deg(optimization_data.uncorrected_slip), 'o');
    ylabel('raw slip [deg]');
    xlabel('roll [deg]')
    
    slip_plots(end+1) = subplot(3,3,9); hold on; grid on; box on;
    plot(optimization_data.airspeed, rad2deg(optimization_data.uncorrected_slip), 'o');
    ylabel('raw slip [deg]');
    xlabel('airspeed [m/s]')
    
    figure('color','w');
    title('AoA Plots')

    aoa_plots(1) = subplot(3,3,1); hold on; grid on; box on;
    plot(optimization_data.g_load, rad2deg(optimization_data.aoa_bias), 'o');
    ylabel('aoa bias [deg]');
    xlabel('g loading [-]')

    aoa_plots(end+1) = subplot(3,3,2); hold on; grid on; box on;
    plot(optimization_data.roll, rad2deg(optimization_data.aoa_bias), 'o');
    ylabel('aoa bias [deg]');
    xlabel('roll [deg]')
    
    aoa_plots(end+1) = subplot(3,3,3); hold on; grid on; box on;
    plot(optimization_data.airspeed, rad2deg(optimization_data.aoa_bias), 'o');
    ylabel('aoa bias [deg]');
    xlabel('airspeed [m/s]')
    
    aoa_plots(end+1) = subplot(3,3,4); hold on; grid on; box on;
    plot(optimization_data.throttle, rad2deg(optimization_data.aoa_bias), 'o');
    ylabel('aoa bias [deg]');
    xlabel('throttle [-]')
    
    aoa_plots(end+1) = subplot(3,3,5); hold on; grid on; box on;
    plot(optimization_data.g_load, rad2deg(optimization_data.aoa), 'o');
    ylabel('corrected aoa [deg]');
    xlabel('g loading [-]')
    
    aoa_plots(end+1) = subplot(3,3,6); hold on; grid on; box on;
    plot(optimization_data.airspeed, rad2deg(optimization_data.aoa), 'o');
    ylabel('corrected aoa [deg]');
    xlabel('airspeed [m/s]')
    
    aoa_plots(end+1) = subplot(3,3,7); hold on; grid on; box on;
    plot(optimization_data.throttle, rad2deg(optimization_data.aoa), 'o');
    ylabel('corrected aoa [deg]');
    xlabel('throttle [-]')
    
    aoa_plots(end+1) = subplot(3,3,8); hold on; grid on; box on;
    plot(optimization_data.g_load, rad2deg(optimization_data.uncorrected_aoa), 'o');
    ylabel('raw aoa [deg]');
    xlabel('g loading [-]')
    
    aoa_plots(end+1) = subplot(3,3,9); hold on; grid on; box on;
    plot(optimization_data.airspeed, rad2deg(optimization_data.uncorrected_aoa), 'o');
    ylabel('raw aoa [deg]');
    xlabel('airspeed [m/s]')
    
    figure('color','w')
    title('Wind Estimate Plots')

    wind_plots(1) = subplot(2,2,1); hold on; grid on; box on;
    plot(optimization_data.wn, 'o');
    plot(optimization_data.we, 'o');
    plot(optimization_data.wd, 'o');
    ylabel('wind speed [m/s]');
    xlabel('loiter instance [-]')
    legend('wn','we','wd');
    
    wind_plots(end+1) = subplot(2,2,2); hold on; grid on; box on;
    plot(rad2deg(optimization_data.roll), optimization_data.wn, 'o');
    plot(rad2deg(optimization_data.roll), optimization_data.we, 'o');
    ylabel('wind speed [m/s]');
    xlabel('roll [deg]')
    legend('wn','we');

    wind_plots(end+1) = subplot(2,2,3); hold on; grid on; box on;
    plot(optimization_data.airspeed, optimization_data.wn, 'o');
    plot(optimization_data.airspeed, optimization_data.we, 'o');
    ylabel('wind speed [m/s]');
    xlabel('airspeed [m/s]')
    legend('wn','we');
    
    wind_plots(end+1) = subplot(2,2,4); hold on; grid on; box on;
    plot(optimization_data.throttle, optimization_data.wn, 'o');
    plot(optimization_data.throttle, optimization_data.we, 'o');
    ylabel('wind speed [m/s]');
    xlabel('throttle [-]')
    legend('wn','we');
    
    try
        figure('color','w'); hold on; grid on; box on;
        title('Slip Bias - Throttle - Roll')
        [xq,yq] = meshgrid(min(optimization_data.throttle):.01:max(optimization_data.throttle), min(rad2deg(optimization_data.roll)):1:max(rad2deg(optimization_data.roll)));
        aoa_mesh = griddata(optimization_data.throttle,rad2deg(optimization_data.roll),rad2deg(optimization_data.slip_bias),xq,yq);
        scatter3(optimization_data.throttle, rad2deg(optimization_data.roll), rad2deg(optimization_data.slip_bias))
        mesh(xq,yq,aoa_mesh)
        xlabel('throttle [-]')
        ylabel('roll [deg]')
        zlabel('slip offset [deg]')
    
        figure('color','w'); hold on; grid on; box on;
        title('AoA Bias - Airspeed - G-Load')
        [xq,yq] = meshgrid(min(optimization_data.airspeed):0.1:max(optimization_data.airspeed), min(optimization_data.g_load):0.01:max(optimization_data.g_load));
        aoa_mesh = griddata(optimization_data.airspeed,optimization_data.g_load,rad2deg(optimization_data.aoa_bias),xq,yq);
        scatter3(optimization_data.airspeed, optimization_data.g_load, rad2deg(optimization_data.aoa_bias))
        mesh(xq,yq,aoa_mesh)
        xlabel('airspeed [m/s]')
        ylabel('roll [deg]')
        zlabel('aoa offset [deg]')
    
        figure('color','w'); hold on; grid on; box on;
        title('AoA Bias - Throttle - G-Load')
        [xq,yq] = meshgrid(min(optimization_data.throttle):.01:max(optimization_data.throttle), min(optimization_data.g_load):0.01:max(optimization_data.g_load));
        aoa_mesh = griddata(optimization_data.throttle,optimization_data.g_load,rad2deg(optimization_data.aoa_bias),xq,yq);
        scatter3(optimization_data.throttle, optimization_data.g_load, rad2deg(optimization_data.aoa_bias))
        mesh(xq,yq,aoa_mesh)
        xlabel('throttle [-]')
        ylabel('g-load [-]')
        zlabel('aoa offset [deg]')
    catch
    end
end

%% Fit a curve through the biases

if length(t_starts) > 1
    % aoa as a function of the g_load
    fo = fitoptions('Method','NonlinearLeastSquares', 'StartPoint', [0 1]);
    ft = fittype('P0 + (1 + x) * P1','options',fo);
    [curve1,gof1] = fit(optimization_data.g_load', optimization_data.aoa_bias', ft);
    x1 = linspace(min(optimization_data.g_load),max(optimization_data.g_load));
    y1 = curve1.P0 + (1.0 + x1) * curve1.P1;

    aoa_bias_g_load = curve1.P0 + (1.0 + optimization_data.g_load) * curve1.P1;

    figure('color','w');
    curve_fit_aoa(1) = subplot(3,1,1); hold on; grid on; box on;
    plot(x1,rad2deg(y1))
%     plot(x1,rad2deg(y12))
    plot(optimization_data.g_load, rad2deg(optimization_data.aoa_bias), 'o')
    xlabel('g-load [-]')
    ylabel('aoa bias [deg]')

    curve_fit_aoa(end+1) = subplot(3,1,2); hold on; grid on; box on;
    plot(optimization_data.throttle, rad2deg(optimization_data.aoa_bias - aoa_bias_g_load), 'o')
    xlabel('throttle [-]')
    ylabel('delta aoa bias [deg]')
    
    curve_fit_aoa(end+1) = subplot(3,1,3); hold on; grid on; box on;
    plot(optimization_data.airspeed, rad2deg(optimization_data.aoa_bias - aoa_bias_g_load), 'o')
    xlabel('airspeed [-]')
    ylabel('delta aoa bias [deg]')
    
    % slip as a function of the roll
    fo = fitoptions('Method','NonlinearLeastSquares', 'StartPoint', [1 1 1]);
    ft = fittype('a*tanh(b*x)+c','options',fo);
    
    [curve,gof] = fit(optimization_data.roll', optimization_data.slip_bias', ft);
    
    x2 = linspace(min(optimization_data.roll),max(optimization_data.roll));
    y2 = curve.a .* tanh(curve.b * x2) + curve.c;
    
    slip_bias_g_load = curve.a .* tanh(curve.b * optimization_data.roll) + curve.c;
 
    figure('color','w');
    curve_fit_slip(1) = subplot(3,1,1); hold on; grid on; box on;
    plot(rad2deg(x2),rad2deg(y2))
    plot(rad2deg(optimization_data.roll), rad2deg(optimization_data.slip_bias), 'o')
    xlabel('roll [deg]')
    ylabel('slip bias [deg]')

    curve_fit_slip(end+1) = subplot(3,1,2); hold on; grid on; box on;
    plot(optimization_data.throttle, rad2deg(optimization_data.slip_bias - slip_bias_g_load), 'o')
    xlabel('throttle [-]')
    ylabel('delta slip bias [deg]')
    
    curve_fit_slip(end+1) = subplot(3,1,3); hold on; grid on; box on;
    plot(optimization_data.airspeed, rad2deg(optimization_data.slip_bias - slip_bias_g_load), 'o')
    xlabel('airspeed [-]')
    ylabel('delta slip bias [deg]')
    
    disp('====================================')
    disp('Fitting results')
    disp('====================================')
    disp(['SF = ',num2str(mean(optimization_data.scale_factor)),'; SF(theory) = ',num2str(sf_theory)])
    disp(['Set CAL_AIR_SCALE = ',num2str(mean(optimization_data.scale_factor))])
    disp(' ')
    disp('aoa_bias = P1 * (1 + g_load) + P0')
    disp(['P1 = ', num2str(curve1.P1)])
    disp(['P0 = ', num2str(curve1.P0)])
    disp(' ')
    disp('slip_bias = A * tanh(B * roll) + C')
    disp(['A = ', num2str(curve.a)])
    disp(['B = ', num2str(curve.b)])
    disp(['C = ', num2str(curve.c)])
end
