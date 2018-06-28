function ControlPlots(sysvector, fconv_gpsalt)
% Display the low level controller data.

% cut rate and attitude setpoints
pRef = getsampleusingtime(sysvector('vehicle_rates_setpoint_0.roll'),...
    sysvector('vehicle_attitude_0.yawspeed').Time(1), sysvector('vehicle_attitude_0.yawspeed').Time(end));
pRef.DataInfo.Interpolation = tsdata.interpolation('zoh');
qRef = getsampleusingtime(sysvector('vehicle_rates_setpoint_0.pitch'),...
    sysvector('vehicle_attitude_0.yawspeed').Time(1), sysvector('vehicle_attitude_0.yawspeed').Time(end));
qRef.DataInfo.Interpolation = tsdata.interpolation('zoh');
rRef = getsampleusingtime(sysvector('vehicle_rates_setpoint_0.yaw'),...
    sysvector('vehicle_attitude_0.yawspeed').Time(1), sysvector('vehicle_attitude_0.yawspeed').Time(end));
rRef.DataInfo.Interpolation = tsdata.interpolation('zoh');

rollRef = getsampleusingtime(sysvector('vehicle_attitude_setpoint_0.roll_body'),...
    sysvector('vehicle_attitude_0.yawspeed').Time(1), sysvector('vehicle_attitude_0.yawspeed').Time(end));
rollRef.DataInfo.Interpolation = tsdata.interpolation('zoh');
pitchRef = getsampleusingtime(sysvector('vehicle_attitude_setpoint_0.pitch_body'),...
    sysvector('vehicle_attitude_0.yawspeed').Time(1), sysvector('vehicle_attitude_0.yawspeed').Time(end));
pitchRef.DataInfo.Interpolation = tsdata.interpolation('zoh');
yawRef = getsampleusingtime(sysvector('vehicle_attitude_setpoint_0.yaw_body'),...
    sysvector('vehicle_attitude_0.yawspeed').Time(1), sysvector('vehicle_attitude_0.yawspeed').Time(end));
yawRef.DataInfo.Interpolation = tsdata.interpolation('zoh');

fig1 = figure();
fig1.Name = 'Attitude+Airspd+Alt Control';
nrSubplotSections = 12;
plotmargins.horiz = 0.05;
plotmargins.vert = 0.016;
plotmargins.vert_last = 0.055;

% Mode and state plot
axeshandle(1) = subplot_tight(nrSubplotSections,1,1,[plotmargins.vert plotmargins.horiz]);
hold on;
stairs(sysvector('commander_state_0.main_state').Time, sysvector('commander_state_0.main_state').Data);
ylabel('Mode [-]')
hold off

% Angle plots
[pitch, roll, yaw] = ...
        QuaternionToEuler(sysvector('vehicle_attitude_0.q_0'), sysvector('vehicle_attitude_0.q_1'),...
        sysvector('vehicle_attitude_0.q_2'), sysvector('vehicle_attitude_0.q_3'));

axeshandle(end+1) = subplot_tight(nrSubplotSections,1,2,[plotmargins.vert plotmargins.horiz]);
hold on;
plot(yaw.Time, rad2deg(yaw.Data));
plot(yawRef.Time, rad2deg(yawRef.Data));
hold off;
legend('Yaw Angle', 'Yaw Angle Ref')

axeshandle(end+1) = subplot_tight(nrSubplotSections,1,[3 4],[plotmargins.vert plotmargins.horiz]);
hold on;
plot(roll.Time, rad2deg(roll.Data));
plot(rollRef.Time, rad2deg(rollRef.Data));
plot(pitch.Time, rad2deg(pitch.Data));
plot(pitchRef.Time, rad2deg(pitchRef.Data));
hold off;
legend('Roll Angle', 'Roll Angle Ref', 'Pitch Angle', 'Pitch Ref');    
ylabel('Attitude [deg]')

% rates plot
axeshandle(end+1) = subplot_tight(nrSubplotSections,1,[5 6],[plotmargins.vert plotmargins.horiz]);
hold on;
plot(sysvector('vehicle_attitude_0.rollspeed').Time, rad2deg(sysvector('vehicle_attitude_0.rollspeed').Data));
plot(pRef.Time, rad2deg(pRef.Data));
plot(sysvector('vehicle_attitude_0.pitchspeed').Time, rad2deg(sysvector('vehicle_attitude_0.pitchspeed').Data));
plot(qRef.Time, rad2deg(qRef.Data));
plot(sysvector('vehicle_attitude_0.yawspeed').Time, rad2deg(sysvector('vehicle_attitude_0.yawspeed').Data));
plot(rRef.Time, rad2deg(rRef.Data));
hold off;
legend('p','pRef','q','qRef','r','rRef');
ylabel('Rates [deg/s]')

% actuator output plots
axeshandle(end+1) = subplot_tight(nrSubplotSections,1,[7 8],[plotmargins.vert plotmargins.horiz]);
hold on;
plot(sysvector('actuator_controls_0.control_0').Time, sysvector('actuator_controls_0.control_0').Data);
plot(sysvector('actuator_controls_0.control_1').Time, sysvector('actuator_controls_0.control_1').Data);
plot(sysvector('actuator_controls_0.control_2').Time, sysvector('actuator_controls_0.control_2').Data);
plot(sysvector('actuator_controls_0.control_3').Time, sysvector('actuator_controls_0.control_3').Data);
plot(sysvector('actuator_controls_0.control_4').Time, sysvector('actuator_controls_0.control_4').Data);
legend('u_{ail}','u_{elev}', 'u_{rud}', 'u_{throt}', 'u_{flaps}');
ylabel('Act. outputs []')

% airspeed plots
axeshandle(end+1) = subplot_tight(nrSubplotSections,1,[9],[plotmargins.vert plotmargins.horiz]);
hold on;
plot(sysvector('airspeed_0.true_airspeed_m_s').Time, sysvector('airspeed_0.true_airspeed_m_s').Data);
plot(sysvector('airspeed_0.indicated_airspeed_m_s').Time, sysvector('airspeed_0.indicated_airspeed_m_s').Data);
plot(sysvector('tecs_status_0.airspeedSp').Time, sysvector('tecs_status_0.airspeedSp').Data);
% TODO add here v ref nom, v ref min, v ref max
legend('v_{TAS} [m/s]','v_{IAS} [m/s]', 'v_{TAS} ref[m/s]');
ylabel('Airsp. [m/s]')

% altitude plots
axeshandle(end+1) = subplot_tight(nrSubplotSections,1,[10 11],[plotmargins.vert plotmargins.horiz]);
plot(sysvector('vehicle_global_position_0.alt').Time, sysvector('vehicle_global_position_0.alt').Data);
hold on;
plot(sysvector('tecs_status_0.altitudeSp').Time, sysvector('tecs_status_0.altitudeSp').Data);
plot(sysvector('position_setpoint_triplet_0.current0x2Ealt').Time, sysvector('position_setpoint_triplet_0.current0x2Ealt').Data);
plot(sysvector('vehicle_gps_position_0.alt').Time, sysvector('vehicle_gps_position_0.alt').Data*fconv_gpsalt);
terrain_alt = sysvector('vehicle_global_position_0.terrain_alt').Data;
terrain_alt(sysvector('vehicle_global_position_0.terrain_alt_valid').Data == 0) = NaN;
plot(sysvector('vehicle_global_position_0.terrain_alt').Time, terrain_alt);
legend('Altitude estimate [m]', 'Alt. ref (smoothed)[m]', 'Alt. ref [m]','GPS Alt [m]','Terrain Altitude [m]');
xlabel('Time [s]')
ylabel('Alt. [m]')

% Plot configuration
for i=1:length(axeshandle)-1; set(axeshandle(end-i),'XTickLabel',''); end;
linkaxes(axeshandle(:),'x');
set(axeshandle(:),'XGrid','on','YGrid','on','ZGrid','on');
dcm_obj = datacursormode(fig1);
set(dcm_obj,'UpdateFcn',@HighPrecisionTooltipCallback);

end

