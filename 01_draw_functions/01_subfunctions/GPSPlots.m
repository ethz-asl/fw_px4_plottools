%% ************************************************************************
%  GPSPlots (nested function)
%  ************************************************************************
%  Display the gps data from the log file. Assumes that successfully GPS
%  data was logged.
%  TODO: - properly scale the axes, maybe convert it to a local Cartesian
%          frame or plot it in spherical coordinates.
%        - Add a callback to get position and extra information (airspeed,
%          gps velocity from a certain point in the plot.

function GPSPlots(sysvector, topics, fconv_gpsalt, fconv_gpslatlong, plotvector)
% 3D plot of the gps position
fig1 = figure();
fig1.Name = 'GPS Position 3D';

if plotvector.colorModeGPS == 2
    plot3(double(sysvector('vehicle_gps_position_0.lat').Data)*fconv_gpslatlong, ...
        double(sysvector('vehicle_gps_position_0.lon').Data)*fconv_gpslatlong, ...
        double(sysvector('vehicle_gps_position_0.alt').Data)*fconv_gpsalt,'r', ...
        'Linewidth', 2);
    title('GPS Position Data');
else
    lat = sysvector('vehicle_gps_position_0.lat').Data*fconv_gpslatlong;
    lon = sysvector('vehicle_gps_position_0.lon').Data*fconv_gpslatlong;
    alt = sysvector('vehicle_gps_position_0.alt').Data*fconv_gpsalt;
    
    if plotvector.colorModeGPS == 1
        gps_vel = CalcGPSHorizontalVelocity(sysvector, topics);
        color = gps_vel.Data;
        title('GPS Position Data (Colored by the horizontal GPS velocity [m/s])');
    elseif plotvector.colorModeGPS == 3
        color = sysvector('vehicle_gps_position_0.vel_m_s').Data;
        title('GPS Position Data (Colored by the GPS velocity [m/s])');
    elseif plotvector.colorModeGPS == 4
        if ~topics.airspeed.logged
           error('The airspeed topic is not logged which is required for plotvector.colorModeGPS == 4')
        end
        
        title('GPS Position Data (Colored by the GPS velocity [m/s])');
        % interpolate airspeed
        airspeed = sysvector('airspeed_0.true_airspeed_m_s');
        if (airspeed.Time(1) > sysvector('vehicle_gps_position_0.lat').Time(1))
            airspeed = addsample(airspeed, 'Data', airspeed.Data(1), 'Time', ...
                sysvector('vehicle_gps_position_0.lat').Time(1));
        end
        if (airspeed.Time(end) < sysvector('vehicle_gps_position_0.lat').Time(end))
            airspeed = addsample(airspeed, 'Data', airspeed.Data(end), 'Time', ...
                sysvector('vehicle_gps_position_0.lat').Time(end));
        end
        airspeed = resample(airspeed, sysvector('vehicle_gps_position_0.lat').Time);
        color = airspeed.Data;
    else
        color = alt;
        title('GPS Position Data (Colored by the GPS altitude [m])');
    end

    surface([lat(:),lat(:)],[lon(:),lon(:)],[alt(:),alt(:)],[color(:),color(:)],...
            'FaceColor','no',...
            'EdgeColor','interp',...
            'LineWidth',2);
    colorbar
end
xlabel('Latitude [deg]');
ylabel('Longitude [deg]');
zlabel('Altitude above MSL [m]');
grid on

% set axis limits
min_lat = min(sysvector('vehicle_gps_position_0.lat').Data)*fconv_gpslatlong;
min_lon = min(sysvector('vehicle_gps_position_0.lon').Data)*fconv_gpslatlong;
max_lat = max(sysvector('vehicle_gps_position_0.lat').Data)*fconv_gpslatlong;
max_lon = max(sysvector('vehicle_gps_position_0.lon').Data)*fconv_gpslatlong;
diff_plot = max(max_lat-min_lat, max_lon-min_lon);
axis([min_lat+0.5*(max_lat-min_lat-diff_plot) min_lat+0.5*(max_lat-min_lat+diff_plot)...
    min_lon+0.5*(max_lon-min_lon-diff_plot) min_lon+0.5*(max_lon-min_lon+diff_plot) -inf inf])

dcm_obj = datacursormode(fig1);
set(dcm_obj,'UpdateFcn',@HighPrecisionTooltipCallback);

% GPS velocity plot
fig2 = figure();
fig2.Name = 'GPS Velocity';
hold on;
plot(sysvector('vehicle_gps_position_0.vel_n_m_s').Time,sysvector('vehicle_gps_position_0.vel_n_m_s').Data);
plot(sysvector('vehicle_gps_position_0.vel_e_m_s').Time,sysvector('vehicle_gps_position_0.vel_e_m_s').Data);
plot(sysvector('vehicle_gps_position_0.vel_d_m_s').Time,sysvector('vehicle_gps_position_0.vel_d_m_s').Data);
hold off;
title('GPS Velocity (NED frame) [m/sec]');
legend('V_N','V_E','V_D');
grid on;
dcm_obj = datacursormode(fig2);
set(dcm_obj,'UpdateFcn',@HighPrecisionTooltipCallback);

fig3 = figure();
fig3.Name = 'GPS Accuracy/DOP';
raw(1)=subplot(5,2,1);
plot(sysvector('vehicle_gps_position_0.eph').Time,sysvector('vehicle_gps_position_0.eph').Data);
title('GPS horizontal position accuracy (eph) [m]');

raw(2)=subplot(5,2,2);
plot(sysvector('vehicle_gps_position_0.epv').Time,sysvector('vehicle_gps_position_0.epv').Data);
title('GPS vertical position accuracy (epv) [m]');

raw(3)=subplot(5,2,3);
plot(sysvector('vehicle_gps_position_0.hdop').Time,sysvector('vehicle_gps_position_0.hdop').Data);
title('Horizontal dilution of precision (hdop)');

raw(4)=subplot(5,2,4);
plot(sysvector('vehicle_gps_position_0.vdop').Time,sysvector('vehicle_gps_position_0.vdop').Data);
title('Vertical dilution of precision (vdop)');

raw(5)=subplot(5,2,5);
plot(sysvector('vehicle_gps_position_0.fix_type').Time,sysvector('vehicle_gps_position_0.fix_type').Data);
title('GPS fix type');

raw(6)=subplot(5,2,6);
plot(sysvector('vehicle_gps_position_0.satellites_used').Time,sysvector('vehicle_gps_position_0.satellites_used').Data);
title('GPS number of used satellites');

raw(7)=subplot(5,2,7);
plot(sysvector('vehicle_gps_position_0.s_variance_m_s').Time,sysvector('vehicle_gps_position_0.s_variance_m_s').Data);
title('GPS estimated speed variance');

raw(8)=subplot(5,2,8);
plot(sysvector('vehicle_gps_position_0.c_variance_rad').Time,sysvector('vehicle_gps_position_0.c_variance_rad').Data);
title('GPS Estimated course variance');

raw(9)=subplot(5,2,9);
plot(sysvector('vehicle_gps_position_0.noise_per_ms').Time,sysvector('vehicle_gps_position_0.noise_per_ms').Data);
title('GPS Noise per ms');

raw(10)=subplot(5,2,10);
plot(sysvector('vehicle_gps_position_0.jamming_indicator').Time,sysvector('vehicle_gps_position_0.jamming_indicator').Data);
title('GPS Jamming Indicator');

linkaxes(raw(:),'x');
set(raw(:),'XGrid','on','YGrid','on','ZGrid','on');
dcm_obj = datacursormode(fig3);
set(dcm_obj,'UpdateFcn',@HighPrecisionTooltipCallback);
end