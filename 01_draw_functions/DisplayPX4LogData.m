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
            plotvector.controlPlots)
        ControlPlots(sysvector, paramvector, params, fconv_gpsalt);
    end

    % display the telemetry data
    if (topics.vehicle_local_position.logged && topics.telemetry_status.logged &&...
            topics.vehicle_attitude.logged && ...
            plotvector.telemRSSIPlots)
        TelemRSSIPlots(sysvector, topics);
    end

    % display the raw sensor data
    if plotvector.rawSensorPlots
       RawSensorPlots(sysvector, topics, fconv_gpsalt);
    end

    % cpu load plots
    if (topics.cpuload.logged && plotvector.cpuLoadPlots)
        CpuLoadPlots(sysvector);
    end

    % distance sensor plots
    if (topics.distance_sensor.logged && plotvector.distanceSensorPlots)
        DistanceSensorPlots(sysvector);
    end

    % mission result plots
    if (topics.mission_result.logged && plotvector.missionResultPlots)
        MissionResultPlots(sysvector);
    end

    % vehicle status flags
    if (topics.vehicle_status_flags.logged && plotvector.vehicleStatusFlags)
        VehicleStatusFlagsPlots(sysvector);
    end

    % mag norm versus thrust plot
    if (topics.actuator_controls_0.logged && topics.sensor_mag.logged && plotvector.magVsThrustPlots)
        MagVsThrustPlots(sysvector);
    end

    % power plots
    if (topics.battery_status.logged && plotvector.powerPlots)
        PowerPlots(sysvector);
    end

    % Battery monitoring plots
    if (topics.sensor_bat_mon.logged && plotvector.BatMonPlots)
         BatMonPlots(sysvector, topics);
    end

    % satcom plots
    if (topics.iridiumsbd_status.logged && plotvector.iridiumsbdStatusPlots)
        IridiumSBDStatusPlots(sysvector);
    end

    % mppt plots
    if (topics.sensor_mppt.logged && plotvector.mpptPlots)
        MpptPlots(sysvector, topics);
    end

    % airflow angle plots
    if ((topics.airflow_aoa.logged || topics.airflow_slip.logged || ...
        topics.sensor_hall.logged) && ...
        plotvector.airflowAnglePlots)
        AirflowAnglePlots(sysvector, topics);
    end

    % link the axes of the different figures
    if plotvector.linkAxes
       LinkFigureAxes();
    end
end