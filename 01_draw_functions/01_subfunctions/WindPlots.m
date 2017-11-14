function WindPlots(sysvector, plotvector)
% Display the wind data.

fig14 = figure(14);
fig14.Name = 'Estimated Wind 3D';
% synchronise the data
dt = plotvector.dtWindPlot;
min_time = max([sysvector('wind_estimate.east').Time(1),...
    sysvector('vehicle_local_position.x').Time(1),...
    sysvector('vehicle_gps_position.vel_n').Time(1),...
    sysvector('airspeed.true_airspeed').Time(1),...
    sysvector('vehicle_attitude.q_0').Time(1)]);
max_time = min([sysvector('wind_estimate.east').Time(end),...
    sysvector('vehicle_local_position.x').Time(end),...
    sysvector('vehicle_gps_position.vel_n').Time(end),...
    sysvector('airspeed.true_airspeed').Time(end),...
    sysvector('vehicle_attitude.q_0').Time(end)]);
time_resampled = min_time:dt:max_time;

pos_x = resample(sysvector('vehicle_local_position.x'), time_resampled);
pos_y = resample(sysvector('vehicle_local_position.y'), time_resampled);
pos_z = resample(sysvector('vehicle_local_position.z'), time_resampled);
wind_e = resample(sysvector('wind_estimate.east'), time_resampled);
wind_n = resample(sysvector('wind_estimate.north'), time_resampled);
vel_n = resample(sysvector('vehicle_gps_position.vel_n'), time_resampled);
vel_e = resample(sysvector('vehicle_gps_position.vel_e'), time_resampled);
vel_d = resample(sysvector('vehicle_gps_position.vel_d'), time_resampled);
airspeed = resample(sysvector('airspeed.true_airspeed'), time_resampled);
q_0 = resample(sysvector('vehicle_attitude.q_0'), time_resampled);
q_1 = resample(sysvector('vehicle_attitude.q_1'), time_resampled);
q_2 = resample(sysvector('vehicle_attitude.q_2'), time_resampled);
q_3 = resample(sysvector('vehicle_attitude.q_3'), time_resampled);

% make z up
pos_z.Data = -pos_z.Data;
vel_d.Data = -vel_d.Data;

plot3(pos_x.Data,pos_y.Data,pos_z.Data,'k.');
axis equal
view(2);
hold all
text(pos_x.Data(1),pos_y.Data(1),pos_z.Data(1),'t_{min}');
text(pos_x.Data(end),pos_y.Data(end),pos_z.Data(end),'t_{max}');
di = numel(pos_x.Data);
distep = floor(di/20);
for i=1:20
    text(pos_x.Data(i*distep),pos_y.Data(i*distep),pos_z.Data(i*distep),num2str(pos_x.Time(i*distep,1)));
end

quiver3(pos_x.Data, pos_y.Data, pos_z.Data,...
        wind_n.Data, wind_e.Data, zeros(size(wind_n.Data)), 0);
if plotvector.plotGroundSpeedVector
    quiver3(pos_x.Data, pos_y.Data, pos_z.Data,...
            vel_n.Data, vel_e.Data, vel_d.Data, 0);
end
if plotvector.plotAirSpeedVector
    [vair_x, vair_y, vair_z] = RotateVector(q_0.Data, q_1.Data, q_2.Data,...
        q_3.Data, airspeed.Data, airspeed.Data*0.0, airspeed.Data*0.0);
    quiver3(pos_x.Data, pos_y.Data, pos_z.Data,...
            vair_x, vair_y, -vair_z, 0);
end
if plotvector.plotGroundSpeedVector && plotvector.plotAirSpeedVector
    legend('pos','v_{wind}','v_{gnd}', 'v_{air}');
elseif plotvector.plotGroundSpeedVector
    legend('pos','v_{wind}','v_{gnd}');
elseif plotvector.plotAirSpeedVector
    legend('pos','v_{wind}','v_{air}');
else
    legend('pos','v_{wind}');
end
title('Wind Data');
xlabel('delta-Longitude [m]');
ylabel('delta-Latitude [m]');
zlabel('Altitude above MSL [m]');
grid on

fig15 = figure(15);
fig15.Name = 'Estimated Wind Plot';
wind(1) = subplot(3,1,1);
hold on;
plot(sysvector('wind_estimate.east').Time, sysvector('wind_estimate.east').Data)
plot(sysvector('wind_estimate.north').Time, sysvector('wind_estimate.north').Data)
legend('wind east', 'wind north')
title('Wind Estimate [m/s]')
hold off;

wind(2) = subplot(3,1,2);
plot(sysvector('wind_estimate.east').Time, ...
    sqrt(sysvector('wind_estimate.east').Data.^2+sysvector('wind_estimate.north').Data.^2))
title('Wind Estimate Magnitude [m/s]')

wind(3) = subplot(3,1,3);
plot(sysvector('wind_estimate.east').Time, ...
    90.0 - atan2d(sysvector('wind_estimate.north').Data, sysvector('wind_estimate.east').Data))
title('Wind Estimate Heading [deg]')

linkaxes(wind(:),'x');
set(wind(:),'XGrid','on','YGrid','on','ZGrid','on');
end

