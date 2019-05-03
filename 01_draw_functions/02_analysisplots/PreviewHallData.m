function [tspan1] = PreviewHallData(sysvector, topics, sensor_sel, tspan, plot_ploy_fit, poly_fit, cal_data)

if (strcmp(sensor_sel, 'ALPHA') && topics.sensor_hall.logged)
    sensor_hall_mag_T = sysvector.sensor_hall_0.mag_T;
elseif (strcmp(sensor_sel, 'BETA') && topics.sensor_hall_01.logged)   
    sensor_hall_mag_T = sysvector.sensor_hall_01_0.mag_T;
else
    clc;
    disp('HALL SENS PREV: Logged topics are not sufficient for airspeed calibration.');
    return;
end

if (plot_ploy_fit && ~isempty(cal_data) && ~isempty(poly_fit))
    
    % check for unpopulated rows..
    last_populated_idx = find(cal_data(:,4) ~= 0 & cal_data(:,5) ~= 0, 1, 'last');

    % / hall plot with poly fit / / / / / / / / / / / / / / / / / / / / / / 
    figure('color','w','name','Hall Sensor Calibration');
    hold on; grid on; box on;
    
    for i = 1:last_populated_idx
        % mT measurements
        ax_(1) = plot(sensor_hall_mag_T.Data(cal_data(i,4):cal_data(i,5)), ...
           ones(length(cal_data(i,4):cal_data(i,5)),1) * cal_data(i,1), '.', 'color', [0.3 0.3 0.3]);
        % mean mT
        ax_(2) = plot(cal_data(i,2), cal_data(i,1), 'o', 'color', 'r');
    end
    % poly fit
    xx = linspace(sensor_hall_mag_T.Data(cal_data(1,4)), ...
        sensor_hall_mag_T.Data(cal_data(last_populated_idx,5)), ...
        501);
    yy = polyval(poly_fit.p, xx);
    ax_(3) = plot(xx, yy, 'linewidth', 2, 'color', 'r');
    
    legend(ax_, {'measurements', 'means', 'poly fit'});
    ylabel('Angle [deg]');
    xlabel('Magnetic field strength [mT]');
    xlim_ = sensor_hall_mag_T.Data([cal_data(1,4), cal_data(last_populated_idx,5)])';
    if xlim_(1)>xlim_(2)
        xlim_ = [xlim_(2), xlim_(1)];
    end
    xlim(xlim_);
    
    tspan1 = 0;
    
else

    min_time = max(tspan(1), sensor_hall_mag_T.Time(1));
    max_time = min(tspan(2), sensor_hall_mag_T.Time(end));
    tspan1 = [min_time,max_time];
    
    idx_st = find(sensor_hall_mag_T.Time > min_time, 1, 'first');
    idx_ed = find(sensor_hall_mag_T.Time < max_time, 1, 'last');

    % / hall plot / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    figure('color','w','name','Hall Sensor Data');

    hold on; grid on; box on;
    plot(sensor_hall_mag_T.Time(idx_st:idx_ed), sensor_hall_mag_T.Data(idx_st:idx_ed));
    ylabel('Magnetic field strength [mT]');

    xlabel('Time [s]');
    xlim(tspan1);

end
