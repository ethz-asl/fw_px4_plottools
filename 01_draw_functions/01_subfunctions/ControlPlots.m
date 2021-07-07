function ControlPlots(sysvector, topics, paramvector, params, fconv_gpsalt)
% Display the low level controller data.

lines_ = lines(7);

% cut rate and attitude setpoints
if topics.vehicle_rates_setpoint.logged && topics.vehicle_attitude.logged
    pRef = getsampleusingtime(sysvector.vehicle_rates_setpoint_0.roll,...
        sysvector.vehicle_attitude_0.q_0.Time(1), sysvector.vehicle_attitude_0.q_0.Time(end));
    pRef.DataInfo.Interpolation = tsdata.interpolation('zoh');
    qRef = getsampleusingtime(sysvector.vehicle_rates_setpoint_0.pitch,...
        sysvector.vehicle_attitude_0.q_0.Time(1), sysvector.vehicle_attitude_0.q_0.Time(end));
    qRef.DataInfo.Interpolation = tsdata.interpolation('zoh');
    rRef = getsampleusingtime(sysvector.vehicle_rates_setpoint_0.yaw,...
        sysvector.vehicle_attitude_0.q_0.Time(1), sysvector.vehicle_attitude_0.q_0.Time(end));
    rRef.DataInfo.Interpolation = tsdata.interpolation('zoh');
end

if topics.vehicle_attitude_setpoint.logged && topics.vehicle_attitude.logged
    rollRef = getsampleusingtime(sysvector.vehicle_attitude_setpoint_0.roll_body,...
        sysvector.vehicle_attitude_0.q_0.Time(1), sysvector.vehicle_attitude_0.q_0.Time(end));
    rollRef.DataInfo.Interpolation = tsdata.interpolation('zoh');
    pitchRef = getsampleusingtime(sysvector.vehicle_attitude_setpoint_0.pitch_body,...
        sysvector.vehicle_attitude_0.q_0.Time(1), sysvector.vehicle_attitude_0.q_0.Time(end));
    pitchRef.DataInfo.Interpolation = tsdata.interpolation('zoh');
    yawRef = getsampleusingtime(sysvector.vehicle_attitude_setpoint_0.yaw_body,...
        sysvector.vehicle_attitude_0.q_0.Time(1), sysvector.vehicle_attitude_0.q_0.Time(end));
    yawRef.DataInfo.Interpolation = tsdata.interpolation('zoh');
end

% resample trim params
if topics.actuator_controls_0.logged
    trim_roll_resampled = TimeseriesExtrapolation(paramvector.trim_roll, sysvector.actuator_controls_0_0.control_0);
    trim_pitch_resampled = TimeseriesExtrapolation(paramvector.trim_pitch, sysvector.actuator_controls_0_0.control_0);
    trim_yaw_resampled = TimeseriesExtrapolation(paramvector.trim_yaw, sysvector.actuator_controls_0_0.control_0);
end

%%%%%%%%%%%%%%%%%%%%%%%%
% First Figure: Overall Control
%%%%%%%%%%%%%%%%%%%%%%%%
fig1 = figure();
fig1.Name = 'Attitude+Airspd+Alt Control';
nrSubplotSections = 12;
plotmargins.horiz = 0.05;
plotmargins.vert = 0.016;
plotmargins.vert_last = 0.055;

% Mode and state plot
axeshandle(1) = subplot_tight(nrSubplotSections,1,1,[plotmargins.vert plotmargins.horiz]);
if topics.commander_state.logged
    hold on;
    stairs(sysvector.commander_state_0.main_state.Time, sysvector.commander_state_0.main_state.Data);
    ylabel('Mode [-]')
    hold off
end

% Angle plots
axeshandle(end+1) = subplot_tight(nrSubplotSections,1,2,[plotmargins.vert plotmargins.horiz]);
if topics.vehicle_attitude.logged && topics.vehicle_attitude_setpoint.logged
    hold on;
    [pitch, roll, yaw] = ...
            QuaternionToEuler(sysvector.vehicle_attitude_0.q_0, sysvector.vehicle_attitude_0.q_1,...
            sysvector.vehicle_attitude_0.q_2, sysvector.vehicle_attitude_0.q_3);
    plot(yaw.Time, rad2deg(yaw.Data));
    plot(yawRef.Time, rad2deg(yawRef.Data));
    hold off;
    legend('Yaw Angle', 'Yaw Angle Ref')
end

axeshandle(end+1) = subplot_tight(nrSubplotSections,1,[3 4],[plotmargins.vert plotmargins.horiz]);
if topics.vehicle_attitude_setpoint.logged && topics.vehicle_attitude.logged
    hold on;
    plot(roll.Time, rad2deg(roll.Data));
    plot(rollRef.Time, rad2deg(rollRef.Data));
    plot(pitch.Time, rad2deg(pitch.Data));
    plot(pitchRef.Time, rad2deg(pitchRef.Data));
    hold off;
    legend('Roll Angle', 'Roll Angle Ref', 'Pitch Angle', 'Pitch Ref');    
    ylabel('Attitude [deg]')
end

% rates plot
axeshandle(end+1) = subplot_tight(nrSubplotSections,1,[5 6],[plotmargins.vert plotmargins.horiz]);
vehicle_attitude_rates_found = false;
if topics.vehicle_angular_velocity.logged
    rollspeed = sysvector.vehicle_angular_velocity_0.xyz_0;
    pitchspeed = sysvector.vehicle_angular_velocity_0.xyz_1;
    yawspeed = sysvector.vehicle_angular_velocity_0.xyz_2;
    vehicle_attitude_rates_found = true;
else
    if topics.vehicle_attitude.logged
       if isfield(sysvector.vehicle_attitude_0, 'rollspeed')
           rollspeed = sysvector.vehicle_attitude_0.rollspeed;
           pitchspeed = sysvector.vehicle_attitude_0.pitchspeed;
           yawspeed = sysvector.vehicle_attitude_0.yawspeed;           
           vehicle_attitude_rates_found = true;
       end
    end
end
if topics.vehicle_rates_setpoint.logged && vehicle_attitude_rates_found
    hold on;
    plot(rollspeed.Time, rad2deg(rollspeed.Data));
    plot(pRef.Time, rad2deg(pRef.Data));
    plot(pitchspeed.Time, rad2deg(pitchspeed.Data));
    plot(qRef.Time, rad2deg(qRef.Data));
    plot(yawspeed.Time, rad2deg(yawspeed.Data));
    plot(rRef.Time, rad2deg(rRef.Data));
    hold off;
    legend('p','pRef','q','qRef','r','rRef');
    ylabel('Rates [deg/s]')
end

% actuator controls plots
axeshandle(end+1) = subplot_tight(nrSubplotSections,1,[7 8],[plotmargins.vert plotmargins.horiz]);
if topics.actuator_controls_0.logged
    hold on;
    ref_opacity = 0.5;
    plot(trim_roll_resampled.Time, trim_roll_resampled.Data, '-.', 'color', ...
        lines_(1,:) * ref_opacity + ones(1,3) * (1 - ref_opacity));
    plot(sysvector.actuator_controls_0_0.control_0.Time, sysvector.actuator_controls_0_0.control_0.Data, 'color', lines_(1,:));
    plot(trim_pitch_resampled.Time, trim_pitch_resampled.Data, '-.', 'color', ...
        lines_(2,:) * ref_opacity + ones(1,3) * (1 - ref_opacity));
    plot(sysvector.actuator_controls_0_0.control_1.Time, sysvector.actuator_controls_0_0.control_1.Data, 'color', lines_(2,:));
    plot(trim_yaw_resampled.Time, trim_yaw_resampled.Data, '-.', 'color', ...
        lines_(3,:) * ref_opacity + ones(1,3) * (1 - ref_opacity));
    plot(sysvector.actuator_controls_0_0.control_2.Time, sysvector.actuator_controls_0_0.control_2.Data, 'color', lines_(3,:));
    plot(sysvector.actuator_controls_0_0.control_3.Time, sysvector.actuator_controls_0_0.control_3.Data, 'color', lines_(4,:));
    plot(sysvector.actuator_controls_0_0.control_4.Time, sysvector.actuator_controls_0_0.control_4.Data, 'color', lines_(5,:));
    legend('u_{ail}^{trim}', 'u_{ail}', 'u_{elev}^{trim}', 'u_{elev}', 'u_{rud}^{trim}', 'u_{rud}', 'u_{throt}', 'u_{flaps}');
    ylabel('Act. controls []')
end

% airspeed plots
axeshandle(end+1) = subplot_tight(nrSubplotSections,1,[9],[plotmargins.vert plotmargins.horiz]);
if topics.airspeed.logged && topics.tecs_status.logged
    hold on;
    plot(sysvector.airspeed_0.true_airspeed_m_s.Time, sysvector.airspeed_0.true_airspeed_m_s.Data);
    plot(sysvector.airspeed_0.indicated_airspeed_m_s.Time, sysvector.airspeed_0.indicated_airspeed_m_s.Data);
    if isfield(sysvector.tecs_status_0, 'airspeedsp')
        plot(sysvector.tecs_status_0.airspeedsp.Time, sysvector.tecs_status_0.airspeedsp.Data);
    elseif isfield(sysvector.tecs_status_0, 'true_airspeed_sp')
        plot(sysvector.tecs_status_0.true_airspeed_sp.Time, sysvector.tecs_status_0.true_airspeed_sp.Data);
    end
    % TODO add here v ref nom, v ref min, v ref max
    legend('v_{TAS} [m/s]','v_{IAS} [m/s]', 'v_{TAS} ref[m/s]');
    ylabel('Airsp. [m/s]')
end

% altitude plots
axeshandle(end+1) = subplot_tight(nrSubplotSections,1,[10 11],[plotmargins.vert plotmargins.horiz]);
if (topics.vehicle_global_position.logged && topics.vehicle_gps_position.logged && ...
        topics.tecs_status.logged && topics.position_setpoint_triplet.logged)
    hold on;
    plot(sysvector.vehicle_global_position_0.alt.Time, sysvector.vehicle_global_position_0.alt.Data);
    plot(sysvector.position_setpoint_triplet_0.current_alt.Time, sysvector.position_setpoint_triplet_0.current_alt.Data);
    plot(sysvector.vehicle_gps_position_0.alt.Time, sysvector.vehicle_gps_position_0.alt.Data*fconv_gpsalt);
    terrain_alt = sysvector.vehicle_global_position_0.terrain_alt.Data;
    terrain_alt(sysvector.vehicle_global_position_0.terrain_alt_valid.Data == 0) = NaN;
    plot(sysvector.vehicle_global_position_0.terrain_alt.Time, terrain_alt);
    if isfield(sysvector.tecs_status_0, 'altitudesp')
        plot(sysvector.tecs_status_0.altitudesp.Time, sysvector.tecs_status_0.altitudesp.Data);
    elseif isfield(sysvector.tecs_status_0, 'altitude_sp')
        plot(sysvector.tecs_status_0.altitude_sp.Time, sysvector.tecs_status_0.altitude_sp.Data);
    end
    legend('Altitude estimate [m]', 'Alt. ref [m]','GPS Alt [m]','Terrain Altitude [m]', 'Alt. ref (smoothed)[m]');
    xlabel('Time [s]')
    ylabel('Alt. [m]')
end

% Plot configuration
for i=1:length(axeshandle)-1; set(axeshandle(end-i),'XTickLabel',''); end;
linkaxes(axeshandle(:),'x');
set(axeshandle(:),'XGrid','on','YGrid','on','ZGrid','on');
dcm_obj = datacursormode(fig1);
set(dcm_obj,'UpdateFcn',@HighPrecisionTooltipCallback);


%%%%%%%%%%%%%%%%%%%%%%%%
% Second Figure : TECS
%%%%%%%%%%%%%%%%%%%%%%%%

fig1 = figure();
fig1.Name = 'TECS status';
nrSubplotSections = 14;
plotmargins.horiz = 0.05;
plotmargins.vert = 0.014;
plotmargins.vert_last = 0.055;

% Mode and state plot
axeshandle = zeros(0,0);
axeshandle(1) = subplot_tight(nrSubplotSections,1,1,[plotmargins.vert plotmargins.horiz]);
if topics.commander_state.logged && topics.tecs_status.logged
    hold on;
    stairs(sysvector.commander_state_0.main_state.Time, sysvector.commander_state_0.main_state.Data);
    stairs(sysvector.tecs_status_0.mode.Time, sysvector.tecs_status_0.mode.Data);
    ylabel('Mode [-]')
    legend('Autopilot mode','TECS mode');
    hold off
end

axeshandle(end+1) = subplot_tight(nrSubplotSections,1,[2 3],[plotmargins.vert plotmargins.horiz]);
if topics.vehicle_attitude_setpoint.logged && topics.vehicle_attitude.logged && topics.tecs_status.logged
    hold on;
    plot(roll.Time, rad2deg(roll.Data));
    plot(rollRef.Time, rad2deg(rollRef.Data));
    plot(pitch.Time, rad2deg(pitch.Data));
    plot(pitchRef.Time, rad2deg(pitchRef.Data));
    if isfield(sysvector.tecs_status_0, 'airspeed_filtered')
        plot(sysvector.tecs_status_0.pitch_integ.Time, rad2deg(sysvector.tecs_status_0.pitch_integ.Data ./ (sysvector.tecs_status_0.airspeed_filtered.Data*5*9.81)));
    elseif isfield(sysvector.tecs_status_0, 'true_airspeed_filtered')
        plot(sysvector.tecs_status_0.pitch_integ.Time, rad2deg(sysvector.tecs_status_0.pitch_integ.Data ./ (sysvector.tecs_status_0.true_airspeed_filtered.Data*5*9.81)));
    end
    hold off;
    legend('Roll Angle', 'Roll Angle Ref', 'Pitch Angle', 'Pitch Ref', 'Pitch Ref I');    
    ylabel('Attitude [deg]')
end

% throttle output
axeshandle(end+1) = subplot_tight(nrSubplotSections,1,[4 5],[plotmargins.vert plotmargins.horiz]);
if topics.actuator_controls_0.logged && topics.tecs_status.logged
    hold on;
    plot(sysvector.actuator_controls_0_0.control_3.Time, sysvector.actuator_controls_0_0.control_3.Data);
    plot(sysvector.tecs_status_0.throttle_integ.Time, sysvector.tecs_status_0.throttle_integ.Data);
    plot(sysvector.actuator_controls_0_0.control_4.Time, sysvector.actuator_controls_0_0.control_4.Data);
    legend('u_{throt}', 'u_{throt,I}','u_{flaps}');
    ylabel('Act. outputs []')
end

% airspeed plots
axeshandle(end+1) = subplot_tight(nrSubplotSections,1,[6 7],[plotmargins.vert plotmargins.horiz]);
if topics.airspeed.logged && topics.tecs_status.logged
    hold on;
    plot(sysvector.airspeed_0.true_airspeed_m_s.Time, sysvector.airspeed_0.true_airspeed_m_s.Data);
    plot(sysvector.airspeed_0.indicated_airspeed_m_s.Time, sysvector.airspeed_0.indicated_airspeed_m_s.Data);
    if isfield(sysvector.tecs_status_0, 'airspeed_filtered')
        plot(sysvector.tecs_status_0.airspeed_filtered.Time, sysvector.tecs_status_0.airspeed_filtered.Data);
    elseif isfield(sysvector.tecs_status_0, 'true_airspeed_filtered')
        plot(sysvector.tecs_status_0.true_airspeed_filtered.Time, sysvector.tecs_status_0.true_airspeed_filtered.Data);
    end
    if isfield(sysvector.tecs_status_0, 'airspeedsp')
        plot(sysvector.tecs_status_0.airspeedsp.Time, sysvector.tecs_status_0.airspeedsp.Data);
    elseif isfield(sysvector.tecs_status_0, 'true_airspeed_sp')
        plot(sysvector.tecs_status_0.true_airspeed_sp.Time, sysvector.tecs_status_0.true_airspeed_sp.Data);
    end
    % TODO add here v ref nom, v ref min, v ref max
    legend('v_{TAS} [m/s]', 'v_{IAS} [m/s]', 'v_{TAS} (filtered) [m/s]', 'v_{TAS} ref[m/s]');
    ylabel('Airsp.')
end

% airspeed plots
axeshandle(end+1) = subplot_tight(nrSubplotSections,1,[8 9],[plotmargins.vert plotmargins.horiz]);
if topics.tecs_status.logged
    hold on;
    if isfield(sysvector.tecs_status_0, 'airspeedderivativesp')
        plot(sysvector.tecs_status_0.airspeedderivativesp.Time, sysvector.tecs_status_0.airspeedderivativesp.Data);
        plot(sysvector.tecs_status_0.airspeedderivative.Time, sysvector.tecs_status_0.airspeedderivative.Data);
    elseif isfield(sysvector.tecs_status_0, 'true_airspeed_derivative_sp')
        plot(sysvector.tecs_status_0.true_airspeed_derivative_sp.Time, sysvector.tecs_status_0.true_airspeed_derivative_sp.Data);
        plot(sysvector.tecs_status_0.true_airspeed_derivative.Time, sysvector.tecs_status_0.true_airspeed_derivative.Data);
    end
    legend('dv/dt_{TAS,ref} [m/s²]','dv/dt_{TAS} [m/s²]');
    ylabel('Airsp. der.')
end

% Energy (and rate) errors
axeshandle(end+1) = subplot_tight(nrSubplotSections,1,[10 11],[plotmargins.vert plotmargins.horiz]);
if topics.tecs_status.logged
    hold on;
    if isfield(sysvector.tecs_status_0, 'totalenergyerror')
        plot(sysvector.tecs_status_0.totalenergyerror.Time, sysvector.tecs_status_0.totalenergyerror.Data);
        plot(sysvector.tecs_status_0.totalenergyrateerror.Time, sysvector.tecs_status_0.totalenergyrateerror.Data);
        plot(sysvector.tecs_status_0.energydistributionerror.Time, sysvector.tecs_status_0.energydistributionerror.Data);
        plot(sysvector.tecs_status_0.energydistributionrateerror.Time, sysvector.tecs_status_0.energydistributionrateerror.Data);
    elseif isfield(sysvector.tecs_status_0, 'total_energy_error')
        plot(sysvector.tecs_status_0.total_energy_error.Time, sysvector.tecs_status_0.total_energy_error.Data);
        plot(sysvector.tecs_status_0.total_energy_rate_error.Time, sysvector.tecs_status_0.total_energy_rate_error.Data);
        plot(sysvector.tecs_status_0.energy_distribution_error.Time, sysvector.tecs_status_0.energy_distribution_error.Data);
        plot(sysvector.tecs_status_0.energy_distribution_rate_error.Time, sysvector.tecs_status_0.energy_distribution_rate_error.Data);
    end
    legend('Total energy error','Total energy rate error','Energy distribution error','Energy distrib. rate error');
    ylabel('Energy')
end

% altitude plots
axeshandle(end+1) = subplot_tight(nrSubplotSections,1,[12 13],[plotmargins.vert plotmargins.horiz]);
if topics.tecs_status.logged && topics.vehicle_global_position.logged && topics.position_setpoint_triplet.logged
    hold on;
    plot(sysvector.vehicle_global_position_0.alt.Time, sysvector.vehicle_global_position_0.alt.Data);
    plot(sysvector.position_setpoint_triplet_0.current_alt.Time, sysvector.position_setpoint_triplet_0.current_alt.Data);
    terrain_alt = sysvector.vehicle_global_position_0.terrain_alt.Data;
    terrain_alt(sysvector.vehicle_global_position_0.terrain_alt_valid.Data == 0) = NaN;
    plot(sysvector.vehicle_global_position_0.terrain_alt.Time, terrain_alt);
    if isfield(sysvector.tecs_status_0, 'altitudesp')
        plot(sysvector.tecs_status_0.altitudesp.Time, sysvector.tecs_status_0.altitudesp.Data);
    elseif isfield(sysvector.tecs_status_0, 'altitude_sp')
        plot(sysvector.tecs_status_0.altitude_sp.Time, sysvector.tecs_status_0.altitude_sp.Data);
    end
    legend('Altitude estimate [m]', 'Alt. ref [m]','Terrain Altitude [m]', 'Alt. ref (smoothed)[m]');
    xlabel('Time [s]')
    ylabel('Alt. [m]')
end

% Plot configuration
for i=1:length(axeshandle)-1; set(axeshandle(end-i),'XTickLabel',''); end;
linkaxes(axeshandle(:),'x');
set(axeshandle(:),'XGrid','on','YGrid','on','ZGrid','on');
dcm_obj = datacursormode(fig1);
set(dcm_obj,'UpdateFcn',@HighPrecisionTooltipCallback);

end