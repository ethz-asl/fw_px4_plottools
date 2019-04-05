% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
% Calibrate flow-based differential pressure sensor parameters
% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

% !!-- This script must be run *after running the logconv.m script. --!!

% This script is intended only for calibration of the SDP3x flow-based
% differential pressure sensor's tube parameters. A nonlinear least squares
% regression is solved minimizing ground speed output error assuming a
% constant wind vector over the input data time span.

%% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
% Plot wind reconstructions from (uncalibrated) true airspeed and ground
% speed measurements to approximate initial guess of NE wind components and
% choose a start and end time of appropriate calibration data. Calibration
% data should satisfy the following criteria:
% - in air at nominal airspeed
% - constant altitude
% - loitering with minimal bank for at least 3 cycles
% - wind speeds are assumed to be constant within the seleted data

% start and end times (modify these)
t_st_cal = -1;
t_ed_cal = 10000;

clc;
if (topics.airspeed.logged && topics.vehicle_gps_position.logged && topics.vehicle_attitude.logged)
    [mean_raw_wind, tspan] = WindPlotsRaw(sysvector, topics, [t_st_cal, t_ed_cal]);
    disp(['Mean wind east = ', num2str(mean_raw_wind(1)), ' m/s']);
    disp(['Mean wind north = ', num2str(mean_raw_wind(2)), ' m/s']);
else
    disp('Logged topics are not sufficient for airspeed calibration.');
end

%% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
% Solve nonlinear least squares regression for sensor calibration

% initial guesses on tube parameters diameter and length should be measured
% on the airframe.

% initial guesses
D0 = 2.2/1000;              % tube diameter [m]
L0 = 0.61;                  % tube length [m]
wn0 = mean_raw_wind(2);     % wind speed north [m/s]
we0 = mean_raw_wind(1);     % wind speed east [m/s]
x0 = [D0; L0; wn0; we0];

% bounds
lb = [1/1000; 0; -20; -20];
ub = [2/1000; 1; 20; 20];

% data
clc;
if (topics.differential_pressure.logged && topics.sensor_baro.logged && ...
        topics.vehicle_attitude.logged && topics.vehicle_gps_position.logged && ...
        topics.airspeed.logged)
    
    [xopt, opt_info] = SolveDpParams(sysvector, tspan, x0, lb, ub);

    disp(['D = ',num2str(xopt(1)*1000),' mm']);
    disp(['L = ',num2str(xopt(2)),' m']);
    disp(['wn = ',num2str(xopt(3)),' m/s']);
    disp(['we = ',num2str(xopt(4)),' m/s']);
else
    disp('Logged topics are not sufficient for airspeed calibration.');
end