% This Matlab Script can be used to import the binary logged values of the
% PX4FMU into data that can be plotted and analyzed.
% A maximum of 6 instances per topics are supported while converting. For
% plotting most plots only display the first instance.
% The script assumes that the topics instances exist in increasing order
% starting from 0.
% TODO:
% - properly catch all cases of a missing topic
% - properly support multi instance topics

%% ************************************************************************
% logconv: Main function
% ************************************************************************
function logconv()
% Clear everything
clc;
clear;
close all;

addpath(genpath('01_draw_functions'));
addpath(genpath('02_helper_functions'));
addpath(genpath('03_kmltoolbox_v2.6'));
addpath(genpath('04_log_files'));
addpath(genpath('05_csv_files'));
addpath(genpath('06_mat_files'));
addpath(genpath('07_kmz_files'));

% ************************************************************************
% SETTINGS (modify necessary parameter)
% ************************************************************************

% set the path to your log file here file here, the log file must be in the
% 04_log_files or one of its subfolders. If it is located in a subfolder
% the path from the level of the 04_log_files needs to be specified here.
fileName = 'log001.ulg';

% the source from which the data is imported
% 0: converting the ulog to csv files and then parsing the csv files
%    (required for the first run)
% 1: only parsing the pre-existing csv files
%    (requires the generated csv files)
% 2: import the data from the .mat file
%    (requires the generated .mat file)
% else: Defaults to 0
loadingMode = 0;

% Print information while converting/loading the log file in mode 0 or 1.
% Helpfull to identify field missmatchs.
loadingVerbose = false;

% indicates if the sysvector map and the topics struct should be saved
% after they are generated.
saveMatlabData = true;

% delete the csv file after a run of the script
deleteCSVFiles = true;

% delimiter for the path
%   '/' for ubuntu
%   '\' for windows
pathDelimiter = '/';

% indicates if the plots should be generated. If set to false the log file
% is only converted to the sysvector.
generatePlots = true;

% only plot the logged data from t_start to t_end. If both are set to 0.0
% all the logged data is plotted.
t_start = 0.0;
t_end = 0.0;

% change topic names or add new topics in the setupTopics function.

% ************************************************************************
% SETTINGS end
% ************************************************************************

% ******************
% Import the data
% ******************

% get the file name without the file ending and path
plainFileName = fileName;

while (contains(plainFileName, pathDelimiter))
    plainFileName = extractAfter(plainFileName, pathDelimiter);
end

plainFileName = char(extractBefore(plainFileName,'.'));

% conversion factors
fconv_timestamp=1E-6;    % [microseconds] to [seconds]
fconv_gpsalt=1E-3;       % [mm] to [m]
fconv_gpslatlong=1E-7;   % [gps_raw_position_unit] to [deg]

if loadingMode==2
    if exist([plainFileName '.mat'], 'file') == 2
        load([plainFileName '.mat']);
        if (numel(fieldnames(topics)) == 0) || (sysvector.Count == 0)
            error(['Sysvector and/or topics loaded from the .mat file are empty.' newline ...
                'Run script first with loadingMode=0 and saveMatlabData=true'])
        end
    else
        error(['Could not load the data as the file does not exist.' newline ...
            'Run script first with loadingMode=0 and saveMatlabData=true'])
    end
else
    % setup the topics which could have been logged
    topics = setupTopics();

    % import the data
    sysvector = containers.Map();
    ImportPX4LogData();
end

% ******************
% Crop the data
% ******************
sysvector_keys = sysvector.keys';
CropPX4LogData();

% ******************
% Print the data
% ******************

if generatePlots
    DisplayPX4LogData(sysvector, topics, plainFileName, fconv_gpsalt, fconv_gpslatlong)
end


%% ************************************************************************
%  *** END OF MAIN SCRIPT ***
%  NESTED FUNCTION DEFINTIONS FROM HERE ON
%  ************************************************************************

%% ************************************************************************
%  ImportPX4LogData (nested function)
%  ************************************************************************
%  Import the data from the log file.

function ImportPX4LogData()
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
                if loadingVerbose
                    str = sprintf('%s\n', string(topics.(topic_fields{idx_topics}).topic_name));
                    fprintf(str)
                end
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
                            str = sprintf('\t%s\n', string(field_name));
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


%% ************************************************************************
%  CropPX4LogData (nested function)
%  ************************************************************************
%  Import the data from the log file.

function CropPX4LogData()
    if (t_start == 0.0 && t_end == 0.0)
        disp('INFO: Not cropping the logging data.')
        return;
    end
    if (t_start > t_end)
        disp('INFO: t_start > t_end: not cropping the logging data.')
        return;
    end
    
    disp('INFO: Start cropping the log data.')
    
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

end
