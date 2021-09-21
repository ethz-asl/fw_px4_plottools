% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
% Calibrate Airspeed Scale Factor
% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

% !!-- This script must be run *after running the logconv.m script. --!!

% A nonlinear least squares regression is solved for constant wind vector
% and airspeed scale factor by minimizing ground speed output errors.

% NOTE: this script is segmented into cells which should be run
%       sequentially after modifying settings at the top of each section

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

% start and end times (modify these)
t_st_cal = 540;
t_ed_cal = 1230;

% ! START do not modify ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
clc;
if (topics.airspeed.logged && topics.vehicle_gps_position.logged && topics.vehicle_attitude.logged)
    [mean_raw_wind, tspan] = WindPlotsRaw(sysvector, topics, [t_st_cal, t_ed_cal]);
    disp(['Mean wind east = ', num2str(mean_raw_wind(1)), ' m/s']);
    disp(['Mean wind north = ', num2str(mean_raw_wind(2)), ' m/s']);
    disp(['Mean wind down = ', num2str(mean_raw_wind(3)), ' m/s']);
else
    disp('ERROR: logged topics are not sufficient for airspeed calibration.');
end
% ! END do not modify ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !

%% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
% Setup optimization (modify these settings for the optimiziation)

% load tube diameter and length from logs (implies these parameters have
% been previously measured and set to the airframe for the given flight)
tube_params_from_logs = true;

% select airframe / pitot configuration (see AirframePitotConfig.m):
% - 'manual-input'
% - 'techpod_long-probe_pre-05-2019'
% - 'ezg3_drotek'
% - 'techpod-agrofly_drotek'
airframe_pitot_config = 'ezg3_drotek';
% NOTE: this config does not affect the resulting calibration - only
%       compares with potential flow theory as a "sanity check"
if strcmp(airframe_pitot_config, 'manual-input')
    % enter config manually
    radius_profile_cm = 1;      % radius of profile on which the probe is mounted (sphere or cylinder) [cm]
    r_probe_tip_cm = 100;       % length of radial vector from center of profile to probe tip [cm]
    theta_probe_tip_deg = 0;    % angle between horizon and radial vector [deg]
    tube_dia = 1.5/1000;        % tube diameter [m] !!-this param is overwritten if loading tube params from logs
    tube_len = 0.5;             % tube length [m]   !!-this param is overwritten if loading tube params from logs
    pitot_type = 1;             % pitot type (drotek pitot = 0; custom pitot = 1)
    mount_location = 1;         % 0 = wing (2D cylinder assumption), 1 = nose (3D sphere assumption)
else
    % load airframe / pitot config from file
    AirframePitotConfig;
end

% ! START do not modify ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
% load tube len/dia from params
if (tube_params_from_logs)
    if (params.cal_air_cmodel.logged && params.cal_air_tubelen.logged && params.cal_air_tubed_mm.logged)
        tube_dia = paramvector.cal_air_tubed_mm.Data(1)/1000;
        tube_len = paramvector.cal_air_tubelen.Data(1);
        if paramvector.cal_air_cmodel.Data(1) == 2
            pitot_type = 1;
        else
            pitot_type = 0;
        end
    else
        disp('ERROR: logged params are not sufficient for airspeed calibration.');    
    end
end

% scale factor from potential flow theory
if (mount_location == 0)
    % potential flow scales (cylinder, 2D)
    sf_theory = 1 / ( ...
        (1 - radius_profile_cm^2 / r_probe_tip_cm^2) * cosd(theta_probe_tip_deg)^2 + ...
        (1 + radius_profile_cm^2 / r_probe_tip_cm^2) * sind(theta_probe_tip_deg)^2 );
elseif (mount_location == 1)
% potential flow scales (sphere, 3D)
    sf_theory = 1 / ( ...
        (1 - radius_profile_cm^3 / r_probe_tip_cm^3) * cosd(theta_probe_tip_deg)^2 + ...
        (1 + radius_profile_cm^3 / r_probe_tip_cm^3 / 2) * sind(theta_probe_tip_deg)^2 );
else
    disp('ERROR: not a valid mounting location');
end
% ! END do not modify ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !

%% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
% Solve nonlinear least squares regression for sensor calibration
% (these settings may be adjusted, but typically do not need to be)

% initial guesses
sf0 = 1;                    % scale factor
ba0 = 0;                    % bias alpha vane [rad]
bs0 = 0;                    % bias slip vane [rad]
wn0 = mean_raw_wind(1);     % wind speed north [m/s]
we0 = mean_raw_wind(2);     % wind speed east [m/s]
x0 = [wn0; we0; sf0; ba0; bs0];

disp(['wn initial = ',num2str(wn0),' m/s']);
disp(['we initial = ',num2str(we0),' m/s']);


% bounds
lb = [-20; -20; 0.1; -0.2; -0.2];
ub = [ 20;  20;   2;  0.2;  0.2];

% ! START do not modify ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
if (topics.differential_pressure.logged && topics.sensor_baro.logged && ...
        topics.vehicle_attitude.logged && topics.vehicle_gps_position.logged && ...
        topics.airspeed.logged && topics.airflow_aoa.logged && topics.airflow_slip.logged && ...
        topics.sensor_gyro.logged)
    
    % solve
    [xopt, opt_info, mean_gsp_err, std_gsp_err, mean_wind_d] = ...
        SolveAirflowCalibration(sysvector, tube_dia, tube_len, pitot_type, tspan, x0, lb, ub);
    
    disp(['wn = ',num2str(xopt(1)),' m/s']);
    disp(['we = ',num2str(xopt(2)),' m/s']);
    disp(['SF = ',num2str(xopt(3)),'; SF(theory) = ',num2str(sf_theory)]);
    disp(['aoa bias = ',num2str(rad2deg(xopt(4))),' deg']);
    disp(['slip bias = ',num2str(rad2deg(xopt(5))),' deg']);
    bounds_exceeded = xopt >= ub | xopt <= lb;
    if sum(bounds_exceeded)>0
        % bounds exceeded 
        disp(['WARNING: optimization bounds exceeded: [',int2str(bounds_exceeded'),']']);
    end
    disp(['Mean of gsp. errors: ',num2str(mean_gsp_err),' m/s']);
    disp(['Standard deviation of gsp. errors: ',num2str(std_gsp_err),' m/s']);
    disp(['Average wind down: ',num2str(mean_wind_d),' m/s']);
    disp(['Set CAL_AIR_SCALE = ',num2str(xopt(3))]);
else
    disp('ERROR: logged topics are not sufficient for airspeed calibration.');
end
% ! END do not modify ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
