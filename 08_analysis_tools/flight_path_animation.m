%% load data
load('/home/acfloria/src/fw_px4_plottools/06_mat_files/EZG3_20210928_Chasseral_flight02.mat')
ezg3 = sysvector;

load('/home/acfloria/src/fw_px4_plottools/06_mat_files/EZG5_20210928_Chasseral_flight02.mat')
ezg5 = sysvector;

load('/home/acfloria/src/fw_px4_plottools/06_mat_files/EZG6_20210928_Chasseral_flight02.mat')
ezg6 = sysvector;

%% resample the data


% resample data
%dp = resample(sysvector.differential_pressure_0.differential_pressure_raw_pa, time_resampled);

t_start_utc = min([...
    ezg3.vehicle_gps_position_0.time_utc_usec.Data(1) * 1e-6,...
    ezg5.vehicle_gps_position_0.time_utc_usec.Data(1) * 1e-6,...
    ezg6.vehicle_gps_position_0.time_utc_usec.Data(1) * 1e-6]);
t_end_utc = max([...
    ezg3.vehicle_gps_position_0.time_utc_usec.Data(end) * 1e-6,...
    ezg5.vehicle_gps_position_0.time_utc_usec.Data(end) * 1e-6,...
    ezg6.vehicle_gps_position_0.time_utc_usec.Data(end) * 1e-6]);

timespan = t_end_utc - t_start_utc;

[~, idx_min] = min(abs(ezg3.vehicle_gps_position_0.time_utc_usec.Data * 1e-6 - t_start_utc));
t_start_ezg3 = ezg3.vehicle_gps_position_0.time_utc_usec.Time(idx_min);

[~, idx_min] = min(abs(ezg5.vehicle_gps_position_0.time_utc_usec.Data * 1e-6 - t_start_utc));
t_start_ezg5 = ezg5.vehicle_gps_position_0.time_utc_usec.Time(idx_min);

[~, idx_min] = min(abs(ezg6.vehicle_gps_position_0.time_utc_usec.Data * 1e-6 - t_start_utc));
t_start_ezg6 = ezg6.vehicle_gps_position_0.time_utc_usec.Time(idx_min);

time_resampled_ezg3 = t_start_ezg3:1:t_start_ezg3+timespan;
ezg3_lat = resample(ezg3.vehicle_global_position_0.lat, time_resampled_ezg3);
ezg3_lon = resample(ezg3.vehicle_global_position_0.lon, time_resampled_ezg3);
ezg3_alt = resample(ezg3.vehicle_global_position_0.alt, time_resampled_ezg3);

time_resampled_ezg5 = t_start_ezg5:1:t_start_ezg5+timespan;
ezg5_lat = resample(ezg5.vehicle_global_position_0.lat, time_resampled_ezg5);
ezg5_lon = resample(ezg5.vehicle_global_position_0.lon, time_resampled_ezg5);
ezg5_alt = resample(ezg5.vehicle_global_position_0.alt, time_resampled_ezg5);

time_resampled_ezg6 = t_start_ezg6:1:t_start_ezg6+timespan;
ezg6_lat = resample(ezg6.vehicle_global_position_0.lat, time_resampled_ezg6);
ezg6_lon = resample(ezg6.vehicle_global_position_0.lon, time_resampled_ezg6);
ezg6_alt = resample(ezg6.vehicle_global_position_0.alt, time_resampled_ezg6);

% convert to WGS84
[ezg3_x, ezg3_y, ezg3_z] = WGS84_to_LV03(ezg3_lat.Data, ezg3_lon.Data, ezg3_alt.Data, true);
[ezg5_x, ezg5_y, ezg5_z] = WGS84_to_LV03(ezg5_lat.Data, ezg5_lon.Data, ezg5_alt.Data, true);
[ezg6_x, ezg6_y, ezg6_z] = WGS84_to_LV03(ezg6_lat.Data, ezg6_lon.Data, ezg6_alt.Data, true);

%% load the terrain
terrain_available = true;
stride_terrain = 15;
if isfile('/home/acfloria/src/fw_sysid/chasseral.tif')
    [map, map_info] = geotiffread('/home/acfloria/src/fw_sysid/chasseral.tif');

    x_map = map_info.XWorldLimits(1) + (map_info.XIntrinsicLimits(1):(map_info.XIntrinsicLimits(2) - 1.0)) * map_info.CellExtentInWorldX;
    y_map = map_info.YWorldLimits(1) + (map_info.YIntrinsicLimits(1):(map_info.YIntrinsicLimits(2) - 1.0)) * map_info.CellExtentInWorldY;

    height_map = map(1:stride_terrain:end, 1:stride_terrain:end);
    x_map = x_map(1:stride_terrain:end);
    y_map = y_map(1:stride_terrain:end);
    
    if map_info.ColumnsStartFrom == 'north'
        height_map = flip(height_map, 1);
    end
    
    if map_info.RowsStartFrom == 'east'
        height_map = flip(height_map, 2);
    end
    
    clear map map_info;
else
    disp('Terrain file does not exist')
    terrain_available = false;
end

if terrain_available
    % get the extent of the flight logs
    extent_x = [min([min(ezg3_x),min(ezg5_x),min(ezg6_x)])-100, max([max(ezg3_x),max(ezg5_x),max(ezg6_x)])+100];
    extent_y = [min([min(ezg3_y),min(ezg5_y),min(ezg6_y)])-100, max([max(ezg3_y),max(ezg5_y),max(ezg6_y)])+100];

    
    % copy the cropped terrain patch
    [~,idx_min_x] = min(abs(x_map - extent_x(1)));
    [~,idx_max_x] = min(abs(x_map - extent_x(2)));
    [~,idx_min_y] = min(abs(y_map - extent_y(1)));
    [~,idx_max_y] = min(abs(y_map - extent_y(2)));
    
    height_map_cropped = height_map(idx_min_y:idx_max_y, idx_min_x:idx_max_x);
    x_map_cropped = x_map(idx_min_x:idx_max_x);
    y_map_cropped = y_map(idx_min_y:idx_max_y);
    
    [X_map_cropped, Y_map_cropped] = meshgrid(x_map_cropped, y_map_cropped);
    
    clear extent_x extent_y idx_min_x idx_max_x idx_min_y idx_max_y
end



%% animate the data
fh = figure();
fh.WindowState = 'maximized';
title('Flight Paths')
hold on;
axis equal;
surf(X_map_cropped, Y_map_cropped, height_map_cropped)

% plot3(ezg3_x, ezg3_y, ezg3_z, 'Linewidth', 2, 'DisplayName', 'EZG3');
% plot3(ezg5_x, ezg5_y, ezg5_z, 'Linewidth', 2, 'DisplayName', 'EZG5');
% plot3(ezg6_x, ezg6_y, ezg6_z, 'Linewidth', 2, 'DisplayName', 'EZG6');
view(-37.7452,8)


h_ezg3 = animatedline('LineWidth', 2, 'Color', 'blue');
h_ezg5 = animatedline('LineWidth', 2, 'Color', 'green');
h_ezg6 = animatedline('LineWidth', 2, 'Color', 'yellow');

pos_ezg3 = scatter3(NaN,NaN,NaN, 40, 'k', 'filled');
pos_ezg5 = scatter3(NaN,NaN,NaN, 40, 'k', 'filled');
pos_ezg6 = scatter3(NaN,NaN,NaN, 40, 'k', 'filled');

drawnow
zlim('manual')

for k = 1:length(time_resampled_ezg6)
    addpoints(h_ezg3,ezg3_x(k),ezg3_y(k), ezg3_z(k));
    addpoints(h_ezg5,ezg5_x(k),ezg5_y(k), ezg5_z(k));
    addpoints(h_ezg6,ezg6_x(k),ezg6_y(k), ezg6_z(k));
    
    pos_ezg3.XData = ezg3_x(k);
    pos_ezg3.YData = ezg3_y(k);
    pos_ezg3.ZData = ezg3_z(k);
    pos_ezg5.XData = ezg5_x(k);
    pos_ezg5.YData = ezg5_y(k);
    pos_ezg5.ZData = ezg5_z(k);
    pos_ezg6.XData = ezg6_x(k);
    pos_ezg6.YData = ezg6_y(k);
    pos_ezg6.ZData = ezg6_z(k);
    drawnow
    saveas(fh,['/tmp/', sprintf( '%06d', k ), '.png'])
end