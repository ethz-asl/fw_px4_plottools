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

% Choose sensor instance
sensor_instance = 0;

% start and end times (modify these if necessary)
t_st_preview = -1;
t_ed_preview = 10000;

% preview the hall data
PreviewHallData(sysvector, topics, sensor_instance, [t_st_preview, t_ed_preview], false, [], []);

%% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
% / Populate calibration array / / / / / / / / / / / / / / / / / / / / / /

% !!-- These values likely require some tuning on your part. --!!

% input the calibration start time (corresponding to angle_st); obtain from
% the previewed plot.
cal_opt.t_st_cal = 0;       % s

% input start and end angles for the calibration
cal_opt.angle_st = -24;         % deg
cal_opt.angle_ed = 24;          % deg

% discretization
cal_opt.large_step_size = 4;    % deg
cal_opt.small_step_size = 2;    % deg
cal_opt.small_step_range = 12;   % deg (this is the +/- range containing the small steps)

% sample weights
cal_opt.use_weighting = true;   % flag to enable the different weighting of the samples
cal_opt.weight_increase = 4.0;  % increased weight for the samples in the small step size range

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
% stuff to play with to make the automated data processing choose the
% segments you want ...

% method to determine the steps and which data should be used for the
% calibration
% Detection Mode 0:
%  This method tries to detect steps based on the filtered derivation of
%  the raw hall sensor measurements. It rejects measurement steps which
%  are too short.
% Detection Mode 1:
%  This method first filters the raw measurements with a moving average
%  filter. Then the standard deviation on those filtered measurements is
%  computed. It registers a measurement as a period of time where the
%  standard deviation is below a certain threshold given that the duration
%  is above the time threshold.

cal_opt.step_detection_mode = 1;


% ----------------
% Options for both detection modes

% minimum step time (tune this to make sure all calibration steps are
% captured, but small perturbations are not)
cal_opt.t_step_min = 5;       % s

% step change threshold
cal_opt.step_thres = 0.15;      % mT

% ----------------
% Options for detection mode 0

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

% ----------------
% Options for detection mode 1

% window size for the moving average filter
cal_opt.window_size = 50;

% standard deviation threshold
cal_opt.std_dev_threshold = 0.01;      % mT

% ----------------

% populate the calibration array -- CHECK this plot output to make sure the
% results are what you expect.. otherwise tune the above parameters until
% you get what you want.
cal_data = AutomatedHallCalibration(sysvector, topics, sensor_instance, cal_opt);

%% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
% / Fit 3rd order polynomial / / / / / / / / / / / / / / / / / / / / / / /

% TODO: just using mean measurements for now.. could do weighted later from
% standard deviations
if cal_opt.use_weighting
    p= polyfitweighted(cal_data(:,2)', cal_data(:,1)', 3, cal_data(:,6)');
else
    p= polyfit(cal_data(:,2)', cal_data(:,1)', 3);
end

poly_fit.p = p;

% parameters (converted to int) for PX4
disp(['CAL_HALL_P0 = ',int2str(int32(p(4)*1e7))]);
disp(['CAL_HALL_P1 = ',int2str(int32(p(3)*1e7))]);
disp(['CAL_HALL_P2 = ',int2str(int32(p(2)*1e7))]);
disp(['CAL_HALL_P3 = ',int2str(int32(p(1)*1e7))]);

% evaluate the error
pred = polyval(poly_fit.p, cal_data(:,2));
disp('MSE = ' + string(immse(pred, cal_data(:,1))))

%% / plot poly fit / / / / / / / / / / / / / / / / / / / / / / / / / / / /

PreviewHallData(sysvector, topics, sensor_instance, [], true, poly_fit, cal_data);
