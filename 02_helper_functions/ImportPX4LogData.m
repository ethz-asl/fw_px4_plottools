function [sysvector, topics] = ImportPX4LogData(fileName, loadingMode, ...
                                                pathDelimiter, fconv_timestamp, ...
                                                loadingVerbose, saveMatlabData, ...
                                                deleteCSVFiles)
%IMPORTPX4LOGDATA Summary of this function goes here
%   Detailed explanation goes here
    disp('INFO: Start importing the log data.')
    
    if exist(fileName, 'file') ~= 2
        error('Log file does not exist: %s', fileName)
    end

    % *********************************
    % convert the log file to csv files
    % *********************************
    if (loadingMode~=1) && (loadingMode~=2)
        tic;
        system(sprintf('ulog2csv 04_log_files%s%s -o 05_csv_files', pathDelimiter, fileName));
        time_csv_conversion = toc;
        disp(['INFO: Converting the ulog file to csv took ' char(num2str(time_csv_conversion)) ' s.'])
    end
    
    % *********************************
    % unpack the csv files
    % *********************************
    disp('INFO: Starting to import the csv data into matlab.')
    tic;
    topic_fields = fieldnames(topics);
    
    if numel(topic_fields) == 0
        error('No topics specified in the setupTopics() function.') 
    end

    %TODO: parallelize the first loop with parfor if we can see another
    %speed issue
    for idx_topics = 1:numel(topic_fields)
        for idx_instance = 0:5
            csv_file = ...
                [plainFileName '_' topics.(topic_fields{idx_topics}).topic_name...
                '_' char(num2str(idx_instance)) '.csv'];
            if exist(csv_file, 'file') == 2
                try
                    csv_data = readtable(csv_file,'ReadVariableNames',true,'Delimiter',',');
                    csv_fields = csv_data.Properties.VariableNames;

                    for idx = 2:numel(csv_fields)
                        field_name = strrep(csv_fields(idx), '0x5B', '_');
                        field_name = strrep(field_name, '0x5D', '');
                        field_name = strip(field_name, '_');

                        ts = timeseries(table2array(csv_data(:, idx)), ...
                            table2array(csv_data(:, 1))*fconv_timestamp, ...
                            'Name', [topic_fields{idx_topics} '.' char(field_name)]);
                        ts.DataInfo.Interpolation = tsdata.interpolation('zoh');
                        sysvector([topic_fields{idx_topics} '_' char(num2str(idx_instance)) '.' char(field_name)]) = ts;

                        if loadingVerbose
                            str = sprintf('%s', string(field_name));
                            fprintf(str)
                        end
                    end

                    % this update avoids that not subsequent instances are
                    % counted
                    if (topics.(topic_fields{idx_topics}).num_instances == idx_instance)
                        topics.(topic_fields{idx_topics}).num_instances = idx_instance + 1;
                        topics.(topic_fields{idx_topics}).logged = true;
                    end
                catch
                    disp(['Could not process the topic: ' char(topic_fields{idx_topics})]);
                end
            else
                break;
            end
        end
    end
    
    % manually add a value for the commander state with the timestamp of
    % the latest global position estimate as they are used together
    if topics.commander_state.logged && topics.vehicle_global_position.logged
       ts_temp = append(sysvector('commander_state_0.main_state'),...
           timeseries(sysvector('commander_state_0.main_state').Data(end),...
           sysvector('vehicle_global_position_0.lon').Time(end)));
       ts_temp.DataInfo.Interpolation = tsdata.interpolation('zoh');
       ts_temp.Name = 'commander_state_0.main_state';
       sysvector('commander_state_0.main_state') = ts_temp;
    end

    time_csv_import = toc;
    disp(['INFO: Importing the csv data to matlab took ' char(num2str(time_csv_import)) ' s.'])

    % check that we have a nonempy sysvector
    if (loadingMode~=1) && (loadingMode~=2)
        if sysvector.Count == 0
            error(['Empty sysvector: Converted the ulog file to csv and parsed it.' newline ...
                'Contains the logfile any topic specified in the setupTopics() function?'])
        end
    else
        if sysvector.Count == 0
            error(['Empty sysvector: Tried to read directly from the csv files.' newline ...
                'Does any csv file for a topic specified the setupTopics() function exist?'])
        end
    end
    
    % *********************************
    % remove duplicate timestamps
    % *********************************
    sysvec_keys = sysvector.keys;
    for idx_key = 1:numel(sysvec_keys)
        % copy data info
        data_info = sysvector(sysvec_keys{idx_key}).DataInfo;
                
        % remove duplicate timestamps
        [~,idx_unique,~] = unique(sysvector(sysvec_keys{idx_key}).Time,'legacy');
        ts_temp = getsamples(sysvector(sysvec_keys{idx_key}), idx_unique);

        ts_temp.DataInfo = data_info;
        sysvector(sysvec_keys{idx_key}) = ts_temp;
    end
   
    % *********************************
    % save the sysvector and topics struct if requested
    % *********************************
    if saveMatlabData
        save(['06_mat_files' pathDelimiter plainFileName '.mat'], 'sysvector', 'topics');
    end
    
    % *********************************
    % delete the csv files if requested
    % *********************************
    if deleteCSVFiles
        system(sprintf('rm 05_csv_files%s%s_*', pathDelimiter, plainFileName));
    end
    
    disp('INFO: Finished importing the log data.')
end