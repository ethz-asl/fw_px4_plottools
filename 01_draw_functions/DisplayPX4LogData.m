%% ************************************************************************
%  DisplayPX4LogData
%  ************************************************************************
%  Display the data from the log file.
%  TODO: Display the waypoints

function DisplayPX4LogData(sysvector, topics, plainFileName, fconv_gpsalt, fconv_gpslatlong)
% ************************************************************
% Settings start
% ************************************************************

% *********************
% Enable/Disable Plots
% *********************
plotvector.gpsPlots = true;
plotvector.sensorPlots = true;
plotvector.differentialPressurePlots = true;
plotvector.estimatorPlots = true;
plotvector.estimatorStatusPlots = true;
plotvector.globalPositionPlots = true;
plotvector.windPlots = true;
plotvector.controllerPlots = true;
plotvector.telemRSSIPlots = true;
plotvector.rawSensorPlots = true;


% *********************
% Link the Figure Axis Settings
% *********************
% Link the axes of different figure
plotvector.linkAxes = true;

% The figures for which the axes should be linked specified by the figure
% number
plotvector.figures_list = [2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, ...
    15, 16, 17, 18 ,19, 20, 21, 22, 23, 24, 25, 26];


% *********************
% GPS Plot Settings
% *********************
% Color mode for the 3D gps plot
% 0: Colored by altitude
% 1: Colored by GPS horizontal velocity
% 2: Red
% 3: Colored by the GPS velocity
% 4: Colored by the filtered airspeed
% Default: Colored by altitude
plotvector.colorModeGPS = 4;


% *********************
% Global Position Estimate Plot Settings
% *********************
% Color mode for the 3D estimated global position plot
% 0: Colored by the control mode
% 1: Red
% 2: Colored by altitude, not in Google Earth
% 3: Colored by the estimated ground velocity, not in Google Earth
% 4: Colored by the filtered airspeed, not in Google Earth
plotvector.colorModeGlobalPosition = 0;

% Indicates if the GPS position should be plot as a reference
plotvector.plotGPSReference = true;

% Indicates if in Google Earth a projection of the path to the ground
% should be displayed.
plotvector.plotPathShadow = true;

% Indicates if GoogleEarth should be autostarted.
plotvector.autostartGoogleEarth = false;

% *********************
% Differential Pressure Plot Settings
% *********************
% Indicates if the differential pressure and airspeed measurements should
% be corrected by the tube pressure loss according to: https://goo.gl/2d3WGc
plotvector.doPressureCorrection = false;

% Diameter of the pitot tube
plotvector.pressureCorrectionD = 0.003; % [m]

% Length of the pitot tube
plotvector.pressureCorrectionL = 0.65; % [m]

% DP reading of the sensor (101 for SDP3X, 62 for SDP600)
plotvector.pressureCorrectionDPSensor = 59.3314; % [Pa]

% Massflow (4.79e-7 for SDP3X, 6.17e-7 for SDP600)
plotvector.pressureCorrectionMassflow = 4.79e-7; % [kg/s]

% *********************
% Wind Plot Settings
% *********************
% Resampling time interval for displaying the wind vector.
plotvector.dtWindPlot = 0.5;

% Indicates if the ground speed vector should be added to the wind vectors
% plot.
plotvector.plotGroundSpeedVector = false;

% Indicates if the air speed vector should be added to the wind vectors
% plot.
plotvector.plotAirSpeedVector = false;

% ************************************************************
% Settings end
% ************************************************************

% display GPS data if it was logged
if topics.vehicle_gps_position.logged && plotvector.gpsPlots
    GPSPlots(sysvector, topics, fconv_gpsalt, fconv_gpslatlong, plotvector);
end

% display sensor data if it was logged
if topics.sensor_combined.logged && plotvector.sensorPlots
    SensorPlots(sysvector, topics, fconv_gpsalt);
end

% display differential pressure data if it was logged
if topics.differential_pressure.logged && topics.airspeed.logged...
        && plotvector.differentialPressurePlots
    DiffPressPlots(sysvector, topics, plotvector);
end

% display estimator data if it was logged
if (topics.estimator_status.logged || topics.ekf2_timestamps.logged ||...
        topics.ekf2_innovations.logged || topics.vehicle_attitude.logged) && plotvector.estimatorPlots
    EstimatorPlots(sysvector, topics);
end

% display gps and estimate on a map
if (topics.vehicle_gps_position.logged || topics.vehicle_global_position.logged) && plotvector.globalPositionPlots
    GlobalPositionPlots(sysvector, topics, plainFileName, fconv_gpsalt,...
        fconv_gpslatlong, plotvector);
end

% display extended wind estimate
if (topics.vehicle_gps_position.logged && topics.vehicle_local_position.logged &&...
        topics.wind_estimate.logged && plotvector.windPlots)
    WindPlots(sysvector, plotvector);
end

% display the controller data
if (topics.vehicle_attitude.logged && topics.vehicle_attitude_setpoint.logged &&...
        topics.vehicle_rates_setpoint.logged && topics.airspeed.logged &&...
        topics.tecs_status.logged && topics.vehicle_gps_position.logged &&...
        plotvector.controllerPlots)
    ControlPlots(sysvector, fconv_gpsalt);
end

% display the telemetry data
if (topics.vehicle_local_position.logged && topics.telemetry_status.logged &&...
        topics.input_rc.logged && topics.vehicle_attitude.logged && ...
        plotvector.telemRSSIPlots)
    TelemRSSIPlots(sysvector);
end

% display the raw sensor data
if plotvector.rawSensorPlots
   RawSensorPlots(sysvector, topics, fconv_gpsalt);
end

% display estimator status data if it was logged
if (topics.estimator_status.logged) && plotvector.estimatorStatusPlots
    EstimatorStatusPlots(sysvector, topics);
end

% link the axes of the different figures
if plotvector.linkAxes
   LinkFigureAxes(plotvector);
end