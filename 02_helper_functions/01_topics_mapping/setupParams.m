function [params] = setupParams()
% topics mapping for the latest master

params = struct;

% actuator trims
params.trim_roll = ...
    struct('param_name', 'trim_roll', 'logged', false);
params.trim_pitch = ...
    struct('param_name', 'trim_pitch', 'logged', false);
params.trim_yaw = ...
    struct('param_name', 'trim_yaw', 'logged', false);

% airflow vane calibration parameters
params.cal_aoa_off = ...
    struct('param_name', 'cal_aoa_off', 'logged', false);
params.cal_slip_off = ...
    struct('param_name', 'cal_slip_off', 'logged', false);
params.cal_hall_min = ...
    struct('param_name', 'cal_hall_min', 'logged', false);
params.cal_hall_max = ...
    struct('param_name', 'cal_hall_max', 'logged', false);
params.cal_hall_rev = ...
    struct('param_name', 'cal_hall_rev', 'logged', false);
params.cal_hall_p0 = ...
    struct('param_name', 'cal_hall_p0', 'logged', false);
params.cal_hall_p1 = ...
    struct('param_name', 'cal_hall_p1', 'logged', false);
params.cal_hall_p2 = ...
    struct('param_name', 'cal_hall_p2', 'logged', false);
params.cal_hall_p3 = ...
    struct('param_name', 'cal_hall_p3', 'logged', false);
params.cal_hall_01_min = ...
    struct('param_name', 'cal_hall_01_min', 'logged', false);
params.cal_hall_01_max = ...
    struct('param_name', 'cal_hall_01_max', 'logged', false);
params.cal_hall_01_rev = ...
    struct('param_name', 'cal_hall_01_rev', 'logged', false);
params.cal_hall_01_p0 = ...
    struct('param_name', 'cal_hall_01_p0', 'logged', false);
params.cal_hall_01_p1 = ...
    struct('param_name', 'cal_hall_01_p1', 'logged', false);
params.cal_hall_01_p2 = ...
    struct('param_name', 'cal_hall_01_p2', 'logged', false);
params.cal_hall_01_p3 = ...
    struct('param_name', 'cal_hall_01_p3', 'logged', false);

% servo rail PWM config
for i=1:8
    str_ = ['pwm_main_min',int2str(i)];
    params.(str_) = struct('param_name', str_, 'logged', false);
end
for i=1:8
    str_ = ['pwm_main_max',int2str(i)];
    params.(str_) = struct('param_name', str_, 'logged', false);
end
for i=1:8
    str_ = ['pwm_main_rev',int2str(i)];
    params.(str_) = struct('param_name', str_, 'logged', false);
end
for i=1:8
    str_ = ['pwm_main_trim',int2str(i)];
    params.(str_) = struct('param_name', str_, 'logged', false);
end

clearvars str_;
