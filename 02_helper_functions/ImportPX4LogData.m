function [sysvector, topics, paramvector, params] = ...
    ImportPX4LogData(fileName, fileLocation, csvLocation, matLocation, ...
                     loadingMode, pathDelimiter, fconv_timestamp, ...
                     loadingVerbose, saveMatlabData, deleteCSVFiles)
%IMPORTPX4LOGDATA Summary of this function goes here
%   Detailed explanation goes here

    % *********************************************************************
    % IMPORT TOPICS *******************************************************
    % *********************************************************************

    disp('INFO: Start importing the log data.')
    
    % setup the topics struct
    topics = setupTopics();

    % *********************************
    % convert the log file to csv files
    % *********************************
    if (loadingMode~=1) && (loadingMode~=2)
        fullFileName = [fileLocation pathDelimiter fileName '.ulg'];
        if exist(fullFileName, 'file') ~= 2
            error('Log file does not exist: %s', fullFileName)
        end

        tic;
        system(sprintf('ulog2csv %s -o %s', fullFileName, csvLocation));
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
                [fileName '_' topics.(topic_fields{idx_topics}).topic_name...
                '_' char(num2str(idx_instance)) '.csv'];
            
            if exist(csv_file, 'file') == 2
                try
                    csv_data = readtable(csv_file,'ReadVariableNames',true,'Delimiter',',');
                    csv_fields = csv_data.Properties.VariableNames;

                    message = struct;
                    if loadingVerbose
                        str = sprintf('%s: ', string(topics.(topic_fields{idx_topics}).topic_name));
                        fprintf(str)
                    end

                    for idx = 2:numel(csv_fields)
                        field_name = strrep(csv_fields(idx), '0x5B', '_');
                        field_name = strrep(field_name, '0x5D', '');
                        field_name = strip(field_name, 'right', '_');
                        % convert all upper case letters to lower case
                        % since the new firmware does expect lower case
                        % field names
                        field_name = lower(field_name);

                        ts = timeseries(table2array(csv_data(:, idx)), ...
                            table2array(csv_data(:, 1))*fconv_timestamp, ...
                            'Name', [topic_fields{idx_topics} '.' char(field_name)]);
                        ts.DataInfo.Interpolation = tsdata.interpolation('zoh');

                        message.(char(field_name)) = ts;

                        if loadingVerbose
                            str = sprintf('%s ', string(field_name));
                            fprintf(str)
                        end
                    end

                    if loadingVerbose
                        fprintf('\n')
                    end
                    sysvector.([topic_fields{idx_topics} '_' char(num2str(topics.(topic_fields{idx_topics}).num_instances))]) = message;

                    topics.(topic_fields{idx_topics}).num_instances = topics.(topic_fields{idx_topics}).num_instances + 1;
                    topics.(topic_fields{idx_topics}).logged = true;
                catch
                    disp(['Could not process the topic: ' char(topic_fields{idx_topics})]);
                end
            end
        end
    end

    % manually add a value for the commander state with the timestamp of
    % the latest global position estimate as they are used together
    if topics.commander_state.logged && topics.vehicle_global_position.logged
       ts_temp = append(sysvector.commander_state_0.main_state,...
           timeseries(sysvector.commander_state_0.main_state.Data(end),...
           sysvector.vehicle_global_position_0.lon.Time(end)));
       ts_temp.DataInfo.Interpolation = tsdata.interpolation('zoh');
       ts_temp.Name = 'commander_state_0.main_state';
       sysvector.commander_state_0.main_state = ts_temp;
    end

    time_csv_import = toc;
    disp(['INFO: Importing the csv data to matlab took ' char(num2str(time_csv_import)) ' s.'])

    % check that we have a nonempy sysvector
    if (loadingMode~=1) && (loadingMode~=2)
        if numel(fieldnames(sysvector)) == 0
            error(['Empty sysvector: Converted the ulog file to csv and parsed it.' newline ...
                'Contains the logfile any topic specified in the setupTopics() function?'])
        end
    else
        if numel(fieldnames(sysvector)) == 0
            error(['Empty sysvector: Tried to read directly from the csv files.' newline ...
                'Does any csv file for a topic specified the setupTopics() function exist?'])
        end
    end
    
    % *********************************
    % remove duplicate timestamps
    % *********************************
    fieldnames_topics = fieldnames(sysvector);
    for idx_tpc = 1:numel(fieldnames_topics)
        fieldnames_message = fieldnames(sysvector.(fieldnames_topics{idx_tpc}));
        
        for idx_msg = 1:numel(fieldnames_message)
            % copy data info
            data_info = sysvector.(fieldnames_topics{idx_tpc}).(fieldnames_message{idx_msg}).DataInfo;

            % remove duplicate timestamps
            [~,idx_unique,~] = unique(sysvector.(fieldnames_topics{idx_tpc}).(fieldnames_message{idx_msg}).Time,'legacy');
            ts_temp = getsamples(sysvector.(fieldnames_topics{idx_tpc}).(fieldnames_message{idx_msg}), idx_unique);

            ts_temp.DataInfo = data_info;
            sysvector.(fieldnames_topics{idx_tpc}).(fieldnames_message{idx_msg}) = ts_temp;
        end
    end
   
    % *********************************************************************
    % IMPORT PARAMETERS ***************************************************
    % *********************************************************************
    
    disp('INFO: Start importing the log param data.')
    
    % setup the params struct
    params = setupParams();

    % *********************************
    % convert the log file to csv files
    % *********************************
    if (loadingMode~=1) && (loadingMode~=2)
        fullFileName = [fileLocation pathDelimiter fileName '.ulg'];
        if exist(fullFileName, 'file') ~= 2
            error('Log file does not exist: %s', fullFileName)
        end
        fullCSVName = [csvLocation pathDelimiter fileName '_params.csv'];

        tic;
        system(sprintf('ulog_params -t %s %s', fullFileName, fullCSVName));
        time_csv_conversion = toc;
        disp(['INFO: Converting the ulog params to csv took ' char(num2str(time_csv_conversion)) ' s.'])
    end
    
    % *********************************
    % unpack the csv files
    % *********************************
    disp('INFO: Starting to import the csv data into matlab.')
    tic;
    param_fields = fieldnames(params);
    
    if numel(param_fields) == 0
        error('No params specified in the setupParams() function.') 
    end
    
    if exist(fullCSVName, 'file') == 2 % make sure file was created
        % read in the data
        opts = detectImportOptions(fullCSVName, 'NumHeaderLines', 0);
        opts.EmptyLineRule = 'read';
        opts.VariableNamesLine = 0; % Overwrite since older matlab versions sometimes detect the first row as the header
        opts.DataLines = [1 Inf];
        csv_data = readtable(fullCSVName, opts);
        size_csv_data = size(csv_data);

        % convert extra column strings to doubles and group numeric param
        % properties
        if size_csv_data(2) > 2
            if isfloat(csv_data{:,3:size_csv_data(2)})
                extra_cols = csv_data{:,3:size_csv_data(2)};

            else
                try
                    extra_cols = str2double(csv_data{:,3:size_csv_data(2)});
                catch
                    extra_cols = [];
                    error('Could not convert the extra columns of the parameter file')
                end
            end
        else
            extra_cols = [];
        end

        if isfloat(csv_data{:,2})
            first_col = csv_data{:,2};
        else
            try
                first_col = str2double(csv_data{:,2});
            catch
                error('Could not convert the parameter values to double')
            end
        end

        csv_numeric_data = [first_col, extra_cols];

        for idx_data = 1:2:size_csv_data(1) % skip timestamp rows

            for idx_params = 1:numel(param_fields)

                try
                    % check if logged field is in defined param list
                    if strcmpi(string(csv_data{idx_data, 1}), string(param_fields{idx_params}))

                        % set field in parameter struct
                        temp_params = csv_numeric_data(idx_data:idx_data+1,:);
                        temp_params = temp_params(:,~isnan(temp_params(1,:)));
                        paramvector.(param_fields{idx_params}) = ...
                            timeseries(temp_params(1,:)', temp_params(2,:)'*fconv_timestamp);
                        % set logged
                        params.(param_fields{idx_params}).logged = true;

                    end

                catch
                    disp(['Could not process the param: ' char(param_fields{idx_params})]);
                end
            end
        end
    else
        error('Parameter CSV file does not exist: %s', fullCSVName);
    end

    time_csv_import = toc;
    disp(['INFO: Importing the csv data to matlab took ' char(num2str(time_csv_import)) ' s.']);

    % check that we have a nonempy paramvector
    if (loadingMode~=1) && (loadingMode~=2)
        if numel(fieldnames(paramvector)) == 0
            error(['Empty paramvector: Converted the ulog file to csv and parsed it.' newline ...
                'Does the logfile contain any parameters specified in the setupParams() function?'])
        end
    else
        if numel(fieldnames(paramvector)) == 0
            error(['Empty paramvector: Tried to read directly from the csv files.' newline ...
                'Does a csv file containing parameters specified in the setupParams() function exist?'])
        end
    end

    % *********************************
    % save the paramvector and params struct if requested
    % *********************************
    if saveMatlabData
        save([matLocation pathDelimiter fileName '.mat'], 'sysvector', 'topics', 'paramvector', 'params');
    end

    % *********************************
    % delete the csv files if requested
    % *********************************
    if deleteCSVFiles
        system(sprintf('rm %s%s%s_*', csvLocation, pathDelimiter, fileName));
    end

    disp('INFO: Finished importing the log data.')
end