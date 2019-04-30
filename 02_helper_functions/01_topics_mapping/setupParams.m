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
