% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
% Deterime hall sensor calibration curve
% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

% !!-- This script must be run *after running the logconv.m script. --!!

% This script fits a 3rd order polynomial describing the relation between
% magnetic field strength (mT) measured by the hall sensor to the angular
% offset from the boom (deg). Calibration parameters are output in the
% suitable form for use with PX4 firmware.

% INSTRUCTIONS:
% 1) fix vane boom to calibration dial
% 2) connect hall sensor to pixhawk / boot pixhawk
% 3) choose a starting/ending angle (note them down: angle_st, angle_ed)
% 4) move the vane such that the tip of the boom is centered on the
% starting angle (prior to this, it is somewhat helpful to make some rapid
% / large motions with the vane such that it is easier to find the
% calibration starting point in the log data)
% 5) hold the vane still at the starting angle for several seconds, then
% move on directly to the next angles, pausing at each for several seconds,
% until reaching the end angle.
% NOTE: it is advised to take steps of 5 degrees at larger angles, until
% reaching e.g. +/- 5-10 degrees, where the step size shold be reduced
% (e.g. 1 or 2 degrees) in an attempt to give a higher accuracy fit at the
% more common values the vane will take while in flight (see calibration 
% option "cal_opt.small_step_range" in the following steps).
% 6) shut down pixhawk, conver the log file, and work with this script.

%% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
% / Preview hall data / / / / / / / / / / / / / / / / / / / / / / / / / / /

% Choose sensor (ALPHA / BETA, corresponding to sensor_hall /
% sensor_hall_01 topics, respectively). ASL FW-team airflow vane
% convention assumes:
% - ALPHA (angle of attack) on *left wing using topic sensor_hall
% - BETA (sideslip) on *right wing, using topic sensor_hall_01
sensor_sel = 'BETA';

% start and end times (modify these if necessary)
t_st_preview = -1;
t_ed_preview = 10000;

% preview the hall data
PreviewHallData(sysvector, topics, sensor_sel, [t_st_preview, t_ed_preview], false, [], []);

%% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
% / Populate calibration array / / / / / / / / / / / / / / / / / / / / / /

% !!-- These values likely require some tuning on your part. --!!

% input the calibration start time (corresponding to angle_st); obtain from
% the previewed plot.
cal_opt.t_st_cal = 56.02;       % s

% input start and end angles for the calibration
cal_opt.angle_st = -55;         % deg
cal_opt.angle_ed = 60;          % deg

% discretization
cal_opt.large_step_size = 5;    % deg
cal_opt.small_step_size = 1;    % deg
cal_opt.small_step_range = 5;   % deg (this is the +/- range containing the small steps)

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
% stuff to play with to make the automated data processing choose the
% segments you want ... 

% minimum step time (tune this to make sure all calibration steps are
% captured, but small perturbations are not)
cal_opt.t_step_min = 5.5;       % s

% step change threshold
cal_opt.step_thres = 0.25;      % mT

% magnetic field strength "standard" deviation
% this is used for checking if we've stepped
cal_opt.mag_dev = 0.13;          % mT

% filter gain
% this is used in a first order digital filter on the deviation checks
cal_opt.k_filt = 0.9;           % ~

% tail crop (cut off some porition of the end of the data before taking
% mean/st.dev, e.g. to account for filter delay)
% TODO: this could be calculated from the filter gain itself...
cal_opt.idx_ed_crop = 3;

% populate the calibration array -- CHECK this plot output to make sure the
% results are what you expect.. otherwise tune the above parameters until
% you get what you want.
cal_data = AutomatedHallCalibration(sysvector, topics, sensor_sel, cal_opt);

%% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
% / Fit 3rd order polynomial / / / / / / / / / / / / / / / / / / / / / / /

% TODO: just using mean measurements for now.. could do weighted later from
% standard deviations
p= polyfit(cal_data(:,2)', cal_data(:,1)', 3);
poly_fit.p = p;


% parameters (converted to int) for PX4
disp(['CAL_HALL_P0 = ',int2str(int32(p(4)*1e7))]);
disp(['CAL_HALL_P1 = ',int2str(int32(p(3)*1e7))]);
disp(['CAL_HALL_P2 = ',int2str(int32(p(2)*1e7))]);
disp(['CAL_HALL_P3 = ',int2str(int32(p(1)*1e7))]);

%% / plot poly fit / / / / / / / / / / / / / / / / / / / / / / / / / / / /

PreviewHallData(sysvector, topics, sensor_sel, [], true, poly_fit, cal_data);
