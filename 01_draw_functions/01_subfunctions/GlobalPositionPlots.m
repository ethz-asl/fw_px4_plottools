%% ************************************************************************
%  GlobalPositionPlots
%  ************************************************************************
%  Display the global position estimate on a 3D plot and in Google Earth.
%  Assumes that the global position estimate was logged. If available and
%  desired the gps position is plotted as a reference. If the commander
%  state is logged the position estimate curve can be colored according to
%  the control mode.
%  TODO: - properly scale the axes, maybe convert it to a local Cartesian
%          frame or plot it in spherical coordinates.
%        - Add a callback to get position and extra information (airspeed,
%          gps velocity from a certain point in the plot.

function GlobalPositionPlots(sysvector, topics, plainFileName, ...
    fconv_gpsalt, fconv_gpslatlong, plotvector)

% check if the commander_state is non-empty
if topics.commander_state.logged && (plotvector.colorModeGlobalPosition == 0) && (numel(sysvector.commander_state_0.main_state.Time) == 0)
    error('DisplayGlobalPositionLogData: No commander_state message in the requested time period')
end

% resample the position, commander state and gps to 10 Hz
min_time = realmin;
max_time = realmax;
if topics.vehicle_global_position.logged
    min_time = max(min_time, sysvector.vehicle_global_position_0.lat.Time(1));
    max_time = min(max_time, sysvector.vehicle_global_position_0.lat.Time(end));
end
if topics.vehicle_gps_position.logged && plotvector.plotGPSReference
    min_time = max(min_time, sysvector.vehicle_gps_position_0.lat.Time(1));
    max_time = min(max_time, sysvector.vehicle_gps_position_0.lat.Time(end));
end
if topics.commander_state.logged && (plotvector.colorModeGlobalPosition == 0)
    min_time = max(min_time, sysvector.commander_state_0.main_state.Time(1));
    max_time = min(max_time, sysvector.commander_state_0.main_state.Time(end));
end
time_resampled = min_time:0.1:max_time;

if topics.vehicle_gps_position.logged
    gps_lat = resample(sysvector.vehicle_gps_position_0.lat*fconv_gpslatlong, time_resampled);
    gps_lon = resample(sysvector.vehicle_gps_position_0.lon*fconv_gpslatlong, time_resampled);
    gps_alt = resample(sysvector.vehicle_gps_position_0.alt*fconv_gpsalt, time_resampled);
end

if topics.vehicle_global_position.logged
    pos_lat = resample(sysvector.vehicle_global_position_0.lat, time_resampled);
    pos_lon = resample(sysvector.vehicle_global_position_0.lon, time_resampled);
    pos_alt = resample(sysvector.vehicle_global_position_0.alt, time_resampled);
end

switch(plotvector.colorModeGlobalPosition)
    case 0
        if ~topics.commander_state.logged
            error('DisplayGlobalPositionLogData: The commander_state topic needs to be logged for using plotvector.colorModeGlobalPosition = 0')
        end
        % define colors for plotting
        colors_rgb = 1/255.0 * ...
            [255 0 0; 113 0 141; 255 133 11; 20 170 0; 9 9 255; 255 231 19;20 250 60; ...
            20 230 240; 130 0 20; 240 120 240; 110 110 110; 210 210 210; 0 0 0];
        colors_kml = strings(1, numel(colors_rgb(:,1)));
        for iter = 1:numel(colors_rgb(:,1))
            colors_kml(iter) = RGBtoKML(colors_rgb(iter,:));
        end

        % detect mode changes
        commander_state = resample(sysvector.commander_state_0.main_state, time_resampled);

        commander_state_change = find(logical(diff(commander_state.Data)))+1;
        if isempty(commander_state_change)
            commander_state_change = [1; numel(commander_state.Data)];
        else
            if commander_state_change(1) ~= 1
                commander_state_change = [1;commander_state_change];
            end
            if commander_state_change(end) ~= numel(commander_state.Data)
                commander_state_change = [commander_state_change;numel(commander_state.Data)];
            end
        end
    case 1
        % nothing to do, only one color
    case 2
        % color changes by altitude 
        color = pos_alt.Data;
    case 3
        % color changes by velocity
        vel_n = resample(sysvector.vehicle_global_position_0.vel_n, time_resampled);
        vel_e = resample(sysvector.vehicle_global_position_0.vel_e, time_resampled);
        vel_d = resample(sysvector.vehicle_global_position_0.vel_d, time_resampled);
        color = sqrt(vel_n.Data.^2+vel_e.Data.^2+vel_d.Data.^2);
    case 4
        if (~topics.airspeed.logged) || (~topics.vehicle_global_position.logged)
           error('The airspeed and global position topic are required for plotvector.colorModeGlobalPosition == 4')
        end
        
        % interpolate airspeed
        airspeed = sysvector.airspeed_0.true_airspeed_m_s;
        if (airspeed.Time(1) > pos_lat.Time(1))
            airspeed = addsample(airspeed, 'Data', airspeed.Data(1), 'Time', ...
                pos_lat.Time(1));
        end
        if (airspeed.Time(end) < pos_lat.Time(end))
            airspeed = addsample(airspeed, 'Data', airspeed.Data(end), 'Time', ...
                pos_lat.Time(end));
        end
        airspeed = resample(airspeed, pos_lat.Time);
        color = airspeed.Data;
    otherwise
        error('DisplayGlobalPositionLogData: Invalid plotvector.colorModeGlobalPosition.')
end

% 3D plot of the global position estimate
if topics.vehicle_global_position.logged
    fig1 = figure();
    fig1.Name = 'Estimated Position 3D Plot';
    switch(plotvector.colorModeGlobalPosition)
        case 0
            % plot global position estimate
            hold on;
            for idx = 1:(numel(commander_state_change)-1)
                plot3(double(pos_lat.Data(commander_state_change(idx):commander_state_change(idx+1))), ...
                    double(pos_lon.Data(commander_state_change(idx):commander_state_change(idx+1))), ...
                    double(pos_alt.Data(commander_state_change(idx):commander_state_change(idx+1))),...
                    'Color',  colors_rgb(commander_state.Data(commander_state_change(idx))+1,:),'Linewidth', 2);
            end
            
            % plot gps reference if required
            if topics.vehicle_gps_position.logged && plotvector.plotGPSReference
                plot3(double(gps_lat.Data), double(gps_lon.Data), double(gps_alt.Data),...
                        'Color',  [1 1 0] ,'Linewidth', 2);
            end
            % construct the legend
            if topics.vehicle_gps_position.logged && plotvector.plotGPSReference
                h = zeros(14, 1);
            else
                h = zeros(13, 1);
            end
            for idx = 1:13
                h(idx) = plot3(NaN,NaN,NaN,'Color',colors_rgb(idx,:),'Linewidth', 2);         
            end
            
            if topics.vehicle_gps_position.logged && plotvector.plotGPSReference
                h(14) = plot3(NaN,NaN,NaN,'Color',[1 1 0] ,'Linewidth', 2);
                legend(h, {'MANUAL','ALTCTL','POSCTL', 'AUTO_MISSION', 'AUTO_LOITER',...
                    'AUTO_RTL', 'ACRO', 'OFFBOARD', 'STAB', 'RATTITUDE', 'AUTO_TAKEOFF',...
                    'AUTO_LAND', 'AUTO_FOLLOW_TARGET', 'GPS reference'}, 'Interpreter', 'none');
            else
                legend(h, {'MANUAL','ALTCTL','POSCTL', 'AUTO_MISSION', 'AUTO_LOITER',...
                    'AUTO_RTL', 'ACRO', 'OFFBOARD', 'STAB', 'RATTITUDE', 'AUTO_TAKEOFF',...
                    'AUTO_LAND', 'AUTO_FOLLOW_TARGET'}, 'Interpreter', 'none');
            end
            hold off;
            title('Global Position Estimate');

        case 1
            hold on;
            plot3(double(pos_lat.Data), double(pos_lon.Data), double(pos_alt.Data),...
                'Color',  [1 0 0] ,'Linewidth', 2);
            if topics.vehicle_gps_position.logged && plotvector.plotGPSReference
                plot3(double(gps_lat.Data), double(gps_lon.Data), double(gps_alt.Data),...
                        'Color',  [1 1 0] ,'Linewidth', 2);
                legend('Global Position Estimate', 'GPS Reference');
            end
            hold off;
            title('Global Position Estimate');
        case 2
            title('Global Position Estimate (colored by altitude [m])');
            hold on;
            surface([pos_lat.Data(:),pos_lat.Data(:)],[pos_lon.Data(:),pos_lon.Data(:)],...
                [pos_alt.Data(:),pos_alt.Data(:)],[color(:),color(:)],...
                'FaceColor','no', 'EdgeColor','interp', 'LineWidth',2);
            if topics.vehicle_gps_position.logged && plotvector.plotGPSReference
                plot3(double(gps_lat.Data), double(gps_lon.Data), double(gps_alt.Data),...
                        'Color',  [0 0 0] ,'Linewidth', 2);
            end
            hold off;
            colorbar
        case 3
            title('Global Position Estimate (colored by velocity [m/s])');
            hold on;
            surface([pos_lat.Data(:),pos_lat.Data(:)],[pos_lon.Data(:),pos_lon.Data(:)],...
                [pos_alt.Data(:),pos_alt.Data(:)],[color(:),color(:)],...
                'FaceColor','no', 'EdgeColor','interp', 'LineWidth',2);
            if topics.vehicle_gps_position.logged && plotvector.plotGPSReference
                plot3(double(gps_lat.Data), double(gps_lon.Data), double(gps_alt.Data),...
                        'Color',  [0 0 0] ,'Linewidth', 2);
            end
            hold off;
            colorbar
        case 4
            title('Global Position Estimate (colored by airspeed [m/s])');
            hold on;
            surface([pos_lat.Data(:),pos_lat.Data(:)],[pos_lon.Data(:),pos_lon.Data(:)],...
                [pos_alt.Data(:),pos_alt.Data(:)],[color(:),color(:)],...
                'FaceColor','no', 'EdgeColor','interp', 'LineWidth',2);
            if topics.vehicle_gps_position.logged && plotvector.plotGPSReference
                plot3(double(gps_lat.Data), double(gps_lon.Data), double(gps_alt.Data),...
                        'Color',  [0 0 0] ,'Linewidth', 2);
            end
            hold off;
            colorbar
    end
    xlabel('Latitude [deg]');
    ylabel('Longitude [deg]');
    zlabel('Altitude above MSL [m]');
    grid on
    
    % set axis limits
    min_lat = min(pos_lat.Data);
    min_lon = min(pos_lon.Data);
    max_lat = max(pos_lat.Data);
    max_lon = max(pos_lon.Data);
    if topics.vehicle_gps_position.logged && plotvector.plotGPSReference
        min_lat = min(min_lat, min(gps_lat.Data));
        min_lon = min(min_lon, min(gps_lon.Data)); 
        max_lat = max(max_lat, max(gps_lat.Data));
        max_lon = max(max_lon, max(gps_lon.Data)); 
    end
    diff_plot = max(max_lat-min_lat, max_lon-min_lon);
    axis([min_lat+0.5*(max_lat-min_lat-diff_plot) min_lat+0.5*(max_lat-min_lat+diff_plot)...
    min_lon+0.5*(max_lon-min_lon-diff_plot) min_lon+0.5*(max_lon-min_lon+diff_plot) -inf inf])
end

% generate kml file
k = kml([plainFileName '_position_plot']);
k.clear;

% plot curve to the kml
if topics.vehicle_gps_position.logged && plotvector.plotGPSReference
    k.useDegrees;
    k.plot3(double(gps_lon.Data), double(gps_lat.Data), double(gps_alt.Data),...
        'altitudeMode','absolute','lineWidth',5,'lineColor','FFFF0000',...
        'name','Position Referance');
    hold on;
end


if topics.vehicle_global_position.logged
    k.useDegrees;
    switch(plotvector.colorModeGlobalPosition)
        case 0
            for idx = 1:(numel(commander_state_change)-1)
                k.plot3(double(pos_lon.Data(commander_state_change(idx):commander_state_change(idx+1))),...
                    double(pos_lat.Data(commander_state_change(idx):commander_state_change(idx+1))),...
                    double(pos_alt.Data(commander_state_change(idx):commander_state_change(idx+1))),...
                    'altitudeMode','absolute','lineWidth',5,'lineColor',...
                    ['FF' char(colors_kml(commander_state.Data(commander_state_change(idx))+1))],...
                    'name','Position Estimation');
            end
        otherwise
            k.plot3(double(pos_lon.Data), double(pos_lat.Data), double(pos_alt.Data),...
                'altitudeMode','absolute','lineWidth',5,'lineColor','FF00FF00',...
                'name','Position Estimation');
    end
    hold on;
end

if (plotvector.plotPathShadow)
    hold on;
    if topics.vehicle_gps_position.logged && plotvector.plotGPSReference
        k.useDegrees;
        k.plot3(double(gps_lon.Data), double(gps_lat.Data), double(gps_alt.Data),...
            'altitudeMode','clampToGround','lineWidth',5,'lineColor',...
            '5FFF0000', 'name','Position Referance (ground)');
    end
    hold on;

    % path shadow (position clamped to ground)
    if topics.vehicle_global_position.logged
        k.useDegrees;
        switch(plotvector.colorModeGlobalPosition)
            case 0 
                for idx = 1:(numel(commander_state_change)-1)
                    k.plot3(double(pos_lon.Data(commander_state_change(idx):commander_state_change(idx+1))),...
                        double(pos_lat.Data(commander_state_change(idx):commander_state_change(idx+1))),...
                        double(pos_alt.Data(commander_state_change(idx):commander_state_change(idx+1))),...
                        'altitudeMode','clampToGround','lineWidth',5,'lineColor',...
                        ['5F' char(colors_kml(commander_state.Data(commander_state_change(idx))+1))],...
                        'name','Position Estimation (ground)');
                end
            otherwise
                k.plot3(double(pos_lon.Data), double(pos_lat.Data), double(pos_alt.Data),...
                    'altitudeMode','clampToGround','lineWidth',5,'lineColor',...
                    '5F00FF00', 'name','Position Estimation (ground)');
        end
        hold on;
    end
end

% save the kml
k.run;

% try opening it in Google Earth
if plotvector.autostartGoogleEarth
    try
        system(sprintf(['xdg-open ' plainFileName '_position_plot.kmz']));
    catch
        warning('Could not open the generated .kmz file in Google Earth, is it installed?')
    end
end


if topics.vehicle_global_position.logged && topics.vehicle_gps_position.logged
    fig2 = figure();
    fig2.Name = 'Estimated Position 2D Plot';
    pos(1) = subplot(4,1,1);
    hold on;
    plot(pos_lat.Time,pos_lat.Data);
    plot(gps_lat.Time,gps_lat.Data);
    hold off;
    legend('Estimated','GPS');
    title('Pos lat [deg]');
    pos(2) = subplot(4,1,2);
    hold on;
    plot(pos_lon.Time,pos_lon.Data);
    plot(gps_lon.Time,gps_lon.Data);
    hold off;
    legend('Estimated','GPS');
    title('Pos lon [deg]');
    pos(3) = subplot(4,1,3);
    hold on;
    plot(pos_alt.Time,pos_alt.Data);
    plot(gps_alt.Time,gps_alt.Data);
    plot(sysvector.vehicle_air_data_0.baro_alt_meter.Time,sysvector.vehicle_air_data_0.baro_alt_meter.Data);
    hold off;
    legend('Estimated','GPS','Baro');
    title('Pos alt [m]');
    pos(4) = subplot(4,1,4);
    hold on;
    plot(sysvector.vehicle_global_position_0.vel_d.Time,sysvector.vehicle_global_position_0.vel_d.Data);
    legend('Downwards velocity v_d');
    title('Downwards velocity v_d[m/s]');

    linkaxes(pos(:),'x');
    set(pos(:),'XGrid','on','YGrid','on','ZGrid','on');
    dcm_obj = datacursormode(fig2);
    set(dcm_obj,'UpdateFcn',@HighPrecisionTooltipCallback);

end

% move kml file to appropriate folder
movefile([plainFileName '_position_plot.kmz'], '07_kmz_files')