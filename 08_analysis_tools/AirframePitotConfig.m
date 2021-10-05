% / / / / / / / / / / / / / / / / / / / / / / /
% for use with CalibrateAirspeedScale.m script
% > > airframe specific pitot configs
% / / / / / / / / / / / / / / / / / / / / / / /

% arbitrary numbering system
if strcmp(airframe_pitot_config, 'techpod_long-probe_pre-05-2019')
    % TECHPOD (LONG PROBE: pre-05/2019)
    config.radius_profile_cm = 5;
    config.r_probe_tip_cm = 28.6;
    config.theta_probe_tip_deg = 19.4;
    config.tube_dia = 3/1000;          % tube diameter [m]
    config.tube_len = 0.56;            % tube length [m]
    config.pitot_type = 1;             % pitot type (drotek pitot = 0; custom pitot = 1)
    config.mount_location = 1;         % 0 = wing (2D cylinder assumption), 1 = nose (3D sphere assumption)
elseif strcmp(airframe_pitot_config, 'ezg3_drotek')
    % EZG3
    config.radius_profile_cm = 1;
    config.r_probe_tip_cm = 7;
    config.theta_probe_tip_deg = 0;
    config.tube_dia = 1.5/1000;        % tube diameter [m]
    %config.tube_len = 0.035;          % tube length [m] (drotek)
    config.tube_len = 0.205;           % tube length [m] (custom)
    config.pitot_type = 0;             % pitot type (drotek pitot = 0; custom pitot = 1)
    config.mount_location = 0;         % 0 = wing (2D cylinder assumption), 1 = nose (3D sphere assumption)
elseif strcmp(airframe_pitot_config, 'techpod-agrofly_drotek')
    % TECHPOD AGROFLY
    config.radius_profile_cm = 5;
    config.r_probe_tip_cm = 12.5;
    config.theta_probe_tip_deg = 0;
    config.tube_dia = 1.5/1000;        % tube diameter [m]
	%config.tube_len = 0.38;           % tube length [m] (drotek)
    config.tube_len = 0.5;             % tube length [m] (custom)
    config.pitot_type = 1;             % pitot type (drotek pitot = 0; custom pitot = 1)
    config.mount_location = 1;         % 0 = wing (2D cylinder assumption), 1 = nose (3D sphere assumption)
elseif strcmp(airframe_pitot_config, 'manual-input')
else
    disp('ERROR: not a valid airframe/pitot configuration.')
end