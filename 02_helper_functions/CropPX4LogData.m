function [ sysvector ] = CropPX4LogData(sysvector, t_start, t_end)
%CROPPX4LOGDATA Crop the data of the sysvector according to t_start [s] and
%t_end [s]. 

    if (isnan(t_start) || isnan(t_end))
        disp('INFO: Not cropping the logging data.')
        return;
    end
    if (t_start > t_end)
        disp('INFO: t_start > t_end: not cropping the logging data.')
        return;
    end

    disp('INFO: Start cropping the log data.')

    sysvector_keys = sysvector.keys();
    for idx_key = 1:numel(sysvector_keys)
        % copy data info
        data_info = sysvector(sysvector_keys{idx_key}).DataInfo;

        % crop time series
        ts_temp = getsampleusingtime(sysvector(sysvector_keys{idx_key}), t_start, t_end);
        ts_temp.DataInfo = data_info;
        sysvector(sysvector_keys{idx_key}) = ts_temp;
    end

    disp('INFO: Finshed cropping the log data.')
end

