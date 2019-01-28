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

    
    fieldnames_topics = fieldnames(sysvector);
    for idx_tpc = 1:numel(fieldnames_topics)
        fieldnames_message = fieldnames(sysvector.(fieldnames_topics{idx_tpc}));
        
        for idx_msg = 1:numel(fieldnames_message)
            % copy data info
            data_info = sysvector.(fieldnames_topics{idx_tpc}).(fieldnames_message{idx_msg}).DataInfo;

            % crop time series
            ts_temp = getsampleusingtime(sysvector.(fieldnames_topics{idx_tpc}).(fieldnames_message{idx_msg}), t_start, t_end);
            ts_temp.DataInfo = data_info;
            sysvector.(fieldnames_topics{idx_tpc}).(fieldnames_message{idx_msg}) = ts_temp;
        end
    end

    disp('INFO: Finshed cropping the log data.')
end

