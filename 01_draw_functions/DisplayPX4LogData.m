%% ************************************************************************
%  DisplayPX4LogData
%  ************************************************************************
%  Display the data from the log file.
%  TODO: Display the waypoints

function DisplayPX4LogData(sysvector, topics, paramvector, params, plainFileName, fconv_gpsalt, fconv_gpslatlong, plotvector)
    % Output flight summary
    PrintFlightMetricsSummary(sysvector, topics);

    % display GPS data if it was logged
    if plotvector.gpsPlots
        GPSPlots(sysvector, topics, fconv_gpsalt, fconv_gpslatlong, plotvector);
    end

    % display sensor data if it was logged
    if plotvector.sensorPlots
        SensorPlots(sysvector, topics, fconv_gpsalt);
    end

    % display differential pressure data if it was logged
    if plotvector.differentialPressurePlots
        DiffPressPlots(sysvector, topics, plotvector);
    end

    % display estimator data if it was logged
    if plotvector.estimatorPlots
        EstimatorPlots(sysvector, topics);
    end

    % display estimator status data if it was logged
    if plotvector.estimatorStatusPlots
        EstimatorStatusPlots(sysvector, topics);
    end

    % display gps and estimate on a map
    if plotvector.globalPositionPlots
        GlobalPositionPlots(sysvector, topics, plainFileName, fconv_gpsalt,...
            fconv_gpslatlong, plotvector);
    end

    % display extended wind estimate
    if plotvector.windPlots
        WindPlots(sysvector, topics, plotvector);
    end

    % display the controller data
    if plotvector.controlPlots
        ControlPlots(sysvector, topics, paramvector, params, fconv_gpsalt);
    end

    % display the telemetry data
    if plotvector.telemRSSIPlots
        TelemRSSIPlots(sysvector, topics);
    end

    % display the raw sensor data
    if plotvector.rawSensorPlots
       RawSensorPlots(sysvector, topics, fconv_gpsalt);
    end

    % cpu load plots
    if plotvector.cpuLoadPlots
        CpuLoadPlots(sysvector, topics);
    end

    % distance sensor plots
    if plotvector.distanceSensorPlots
        DistanceSensorPlots(sysvector, topics);
    end

    % mission result plots
    if plotvector.missionResultPlots
        MissionResultPlots(sysvector, topics);
    end

    % vehicle status flags
    if plotvector.vehicleStatusFlags
        VehicleStatusFlagsPlots(sysvector, topics);
    end

    % mag norm versus thrust plot
    if plotvector.magVsThrustPlots
        MagVsThrustPlots(sysvector, topics);
    end

    % power plots
    if plotvector.powerPlots
        PowerPlots(sysvector, topics);
    end

    % Battery monitoring plots
    if plotvector.BatMonPlots
         BatMonPlots(sysvector, topics);
    end

    % satcom plots
    if plotvector.iridiumsbdStatusPlots
        IridiumSBDStatusPlots(sysvector, topics);
    end

    % mppt plots
    if plotvector.mpptPlots
        MpptPlots(sysvector, topics);
    end

    % airflow angle plots
    if plotvector.airflowAnglePlots
        AirflowAnglePlots(sysvector, topics);
    end

    % link the axes of the different figures
    if plotvector.linkAxes
       LinkFigureAxes();
    end
end