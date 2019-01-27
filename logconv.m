% This Matlab Script can be used to import the binary logged values of the
% PX4FMU into data that can be plotted and analyzed.
% A maximum of 6 instances per topics are supported while converting. For
% plotting most plots only display the first instance.
% The script assumes that the topics instances exist in increasing order
% starting from 0.
% TODO:
% - properly catch all cases of a missing topic
% - properly support multi instance topics

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

% The log file name, no file specifier required
fileName = 'log001';

% The log file location, only required when converting a .ulg file. All
% .csv files are supposed to be in 05_csv_files, and all .mat files are
% supposed to be in 06_mat_files
fileLocation = '04_log_files';

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
deleteCSVFiles = false;

% delimiter for the path
%   '/' for ubuntu
%   '\' for windows
pathDelimiter = '/';

% indicates if the plots should be generated. If set to false the log file
% is only converted to the sysvector.
generatePlots = true;

% only plot the logged data from t_start to t_end. If one of them is set to
% NaN all the logged data is plotted [s].
t_start = NaN;
t_end = NaN;

% change topic names or add new topics in the setupTopics function.

% ************************************************************************
% SETTINGS end
% ************************************************************************

% ******************
% Import the data
% ******************

% conversion factors
fconv_timestamp=1E-6;    % [microseconds] to [seconds]
fconv_gpsalt=1E-3;       % [mm] to [m]
fconv_gpslatlong=1E-7;   % [gps_raw_position_unit] to [deg]

if loadingMode==2
    if exist([fileName '.mat'], 'file') == 2
        load([fileName '.mat']);
        if (numel(fieldnames(topics)) == 0) || (sysvector.Count == 0)
            error(['Sysvector and/or topics loaded from the .mat file are empty.' newline ...
                'Run script first with loadingMode=0 and saveMatlabData=true'])
        end
    else
        error(['Could not load the data as the file does not exist.' newline ...
            'Run script first with loadingMode=0 and saveMatlabData=true'])
    end
else
    [sysvector, topics] = ImportPX4LogData(fileName, fileLocation, loadingMode, pathDelimiter,...
        fconv_timestamp, loadingVerbose, saveMatlabData, deleteCSVFiles);
end

% ******************
% Crop the data
% ******************

CropPX4LogData(sysvector, t_start, t_end);

% ******************
% Print the data
% ******************

if generatePlots
    DisplayPX4LogData(sysvector, topics, fileName, fconv_gpsalt, fconv_gpslatlong)
end
