function ControlPlots(sysvector, fconv_gpsalt)
% Display the low level controller data.

% cut rate and attitude setpoints
pRef = getsampleusingtime(sysvector('vehicle_rates_setpoint.roll'),...
    sysvector('vehicle_attitude.yawspeed').Time(1), sysvector('vehicle_attitude.yawspeed').Time(end));
pRef.DataInfo.Interpolation = tsdata.interpolation('zoh');
qRef = getsampleusingtime(sysvector('vehicle_rates_setpoint.pitch'),...
    sysvector('vehicle_attitude.yawspeed').Time(1), sysvector('vehicle_attitude.yawspeed').Time(end));
qRef.DataInfo.Interpolation = tsdata.interpolation('zoh');
rRef = getsampleusingtime(sysvector('vehicle_rates_setpoint.yaw'),...
    sysvector('vehicle_attitude.yawspeed').Time(1), sysvector('vehicle_attitude.yawspeed').Time(end));
rRef.DataInfo.Interpolation = tsdata.interpolation('zoh');

rollRef = getsampleusingtime(sysvector('vehicle_attitude_setpoint.roll_body'),...
    sysvector('vehicle_attitude.yawspeed').Time(1), sysvector('vehicle_attitude.yawspeed').Time(end));
rollRef.DataInfo.Interpolation = tsdata.interpolation('zoh');
pitchRef = getsampleusingtime(sysvector('vehicle_attitude_setpoint.pitch_body'),...
    sysvector('vehicle_attitude.yawspeed').Time(1), sysvector('vehicle_attitude.yawspeed').Time(end));
pitchRef.DataInfo.Interpolation = tsdata.interpolation('zoh');
yawRef = getsampleusingtime(sysvector('vehicle_attitude_setpoint.yaw_body'),...
    sysvector('vehicle_attitude.yawspeed').Time(1), sysvector('vehicle_attitude.yawspeed').Time(end));
yawRef.DataInfo.Interpolation = tsdata.interpolation('zoh');

figure('Name', 'Attitude and Rate Control');
% rates plot
control(1) = subplot(4,1,1);
hold on;
plot(sysvector('vehicle_attitude.rollspeed').Time, sysvector('vehicle_attitude.rollspeed').Data);
plot(pRef.Time, pRef.Data);
plot(sysvector('vehicle_attitude.pitchspeed').Time, sysvector('vehicle_attitude.pitchspeed').Data);
plot(qRef.Time, qRef.Data);
plot(sysvector('vehicle_attitude.yawspeed').Time, sysvector('vehicle_attitude.yawspeed').Data);
plot(rRef.Time, rRef.Data);
hold off;
legend('p','pRef','q','qRef','r','rRef');
ylabel('Rates [rad/s]')

% angles plot
[pitch, roll, yaw] = ...
        QuaternionToEuler(sysvector('vehicle_attitude.q_0'), sysvector('vehicle_attitude.q_1'),...
        sysvector('vehicle_attitude.q_2'), sysvector('vehicle_attitude.q_3'));
    
control(2) = subplot(4,1,2);
hold on;
plot(roll.Time, roll.Data);
plot(rollRef.Time, rollRef.Data);
plot(pitch.Time, pitch.Data);
plot(pitchRef.Time, pitchRef.Data);
hold off;
legend('Roll Angle', 'Roll Angle Ref', 'Pitch Angle', 'Pitch Ref');    
ylabel('Attitude [deg]')

% yaw angle plot
control(3) = subplot(4,1,3);
hold on;
plot(yaw.Time, yaw.Data);
plot(yawRef.Time, yawRef.Data);
hold off;
legend('Yaw Angle', 'Yaw Angle Ref')
ylabel('Attitude [deg]')

control(4) = subplot(4,1,4);
% airspeed plots
hold on;
plot(sysvector('airspeed.true_airspeed').Time, sysvector('airspeed.true_airspeed').Data);
plot(sysvector('airspeed.indicated_airspeed').Time, sysvector('airspeed.indicated_airspeed').Data);
plot(sysvector('tecs_status.airspeedSp').Time, sysvector('tecs_status.airspeedSp').Data);
% TODO add here v ref nom, v ref min, v ref max
legend('v_{TAS} [m/s]','v_{IAS} [m/s]', 'v_{IAS} ref[m/s]');
ylabel('Airspeed [m/s]')
hold off;

linkaxes(control(:),'x');
set(control(:),'XGrid','on','YGrid','on','ZGrid','on');


figure('Name', 'Airspeed Control');
% altitude plot
hold on;
plot(sysvector('vehicle_global_position.alt').Time, sysvector('vehicle_global_position.alt').Data);
plot(sysvector('tecs_status.altitudeSp').Time, sysvector('tecs_status.altitudeSp').Data);
plot(sysvector('vehicle_gps_position.alt').Time, sysvector('vehicle_gps_position.alt').Data*fconv_gpsalt);
% TODO add here alt min/max and altitude ramp (if that exists)
legend('Altitude estimate [m]', 'Altitude ref[m]', 'GPS Alt [m]');
xlabel('Time [s]')
ylabel('Altitude [m]')
hold off;
end

