function [aoa, slip, aoa_logged, slip_logged] = CalibrateAirflowAngles(sysvector, topics, paramvector, params, config)
% Calibrates the airflow angles according to the configuration, returns the
% airflow angles with a bias

% XXX: TODO - incorporate temperature calibration
% mag_temp_comp = sysvector.sensor_hall_0.mag_T.Data + ...
%     (sysvector.sensor_hall_0.temp_C.Data - 25) .* 0.0012 .* sysvector.sensor_hall_0.mag_T.Data;

aoa_logged = true;
slip_logged = true;

% check if the old or new calibration params are used
new_params = params.cal_av_aoa_rev.logged;

% set as the default the manual config params
aoa_rev =  config.cal_hall_aoa_rev;
aoa_p0 = config.cal_hall_aoa_p0;
aoa_p1 = config.cal_hall_aoa_p1;
aoa_p2 = config.cal_hall_aoa_p2;
aoa_p3 = config.cal_hall_aoa_p3;
aoa_id = config.cal_hall_aoa_id;
slip_rev =  config.cal_hall_slip_rev;
slip_p0 = config.cal_hall_slip_p0;
slip_p1 = config.cal_hall_slip_p1;
slip_p2 = config.cal_hall_slip_p2;
slip_p3 = config.cal_hall_slip_p3;
slip_id = config.cal_hall_slip_id;

if new_params
    % aoa params
    aoa_params_logged = (params.cal_av_aoa_id.logged && ...
        params.cal_av_aoa_rev.logged && ...
        params.cal_av_aoa_p0.logged && ...
        params.cal_av_aoa_p1.logged && ...
        params.cal_av_aoa_p2.logged && ...
        params.cal_av_aoa_p3.logged);

    if (config.use_cal_av_params && aoa_params_logged)
       aoa_rev =  paramvector.cal_av_aoa_rev.Data(1);
       aoa_p0 = paramvector.cal_av_aoa_p0.Data(1);
       aoa_p1 = paramvector.cal_av_aoa_p1.Data(1);
       aoa_p2 = paramvector.cal_av_aoa_p2.Data(1);
       aoa_p3 = paramvector.cal_av_aoa_p3.Data(1);
       aoa_id = paramvector.cal_av_aoa_id.Data(1);
    end

    % slip params
    slip_params_logged = (params.cal_av_slip_id.logged && ...
        params.cal_av_slip_rev.logged && ...
        params.cal_av_slip_p0.logged && ...
        params.cal_av_slip_p1.logged && ...
        params.cal_av_slip_p2.logged && ...
        params.cal_av_slip_p3.logged);
    
    if (config.use_cal_av_params && slip_params_logged)
       slip_rev =  paramvector.cal_av_slip_rev.Data(1);
       slip_p0 = paramvector.cal_av_slip_p0.Data(1);
       slip_p1 = paramvector.cal_av_slip_p1.Data(1);
       slip_p2 = paramvector.cal_av_slip_p2.Data(1);
       slip_p3 = paramvector.cal_av_slip_p3.Data(1);
       slip_id = paramvector.cal_av_slip_id.Data(1);
    end
    
    % aoa and slip data
    aoa_hall_logged = false;
    slip_hall_logged = false;
    for i = 0:topics.sensor_hall.num_instances - 1
        hall_name = strcat('sensor_hall_', num2str(i));
        
        if sysvector.(hall_name).instance.Data(1) == aoa_id
            aoa_hall_data = sysvector.(hall_name).mag_T;
            aoa_hall_logged = true;
        end

        if sysvector.(hall_name).instance.Data(1) == slip_id
            slip_hall_data = sysvector.(hall_name).mag_T;
            slip_hall_logged = true;
        end
    end
else
    % legacy aoa params
    aoa_params_logged = (params.cal_hall_rev.logged && ...
        params.cal_hall_p0.logged && ...
        params.cal_hall_p1.logged && ...
        params.cal_hall_p2.logged && ...
        params.cal_hall_p3.logged);

    if (config.use_cal_av_params && aoa_params_logged)
       aoa_rev =  paramvector.cal_hall_rev.Data(1);
       aoa_p0 = paramvector.cal_hall_p0.Data(1);
       aoa_p1 = paramvector.cal_hall_p1.Data(1);
       aoa_p2 = paramvector.cal_hall_p2.Data(1);
       aoa_p3 = paramvector.cal_hall_p3.Data(1);
    end
    
    % legacy aoa hall data
    aoa_hall_logged = topics.sensor_hall.logged;
    
    if aoa_hall_logged
        aoa_hall_data = sysvector.sensor_hall_0.mag_T;
    end
    
    % legacy slip params
    slip_params_logged = params.cal_hall_01_rev.logged && ...
            params.cal_hall_01_p0.logged && ...
            params.cal_hall_01_p1.logged && ...
            params.cal_hall_01_p2.logged && ...
            params.cal_hall_01_p3.logged;

    if (config.use_cal_av_params && slip_params_logged)
       slip_rev =  paramvector.cal_hall_01_rev.Data(1);
       slip_p0 = paramvector.cal_hall_01_p0.Data(1);
       slip_p1 = paramvector.cal_hall_01_p1.Data(1);
       slip_p2 = paramvector.cal_hall_01_p2.Data(1);
       slip_p3 = paramvector.cal_hall_01_p3.Data(1);
    end

    % legacy slip hall data
    slip_hall_logged = topics.sensor_hall_01.logged;
    
    if slip_hall_logged
        slip_hall_data = sysvector.sensor_hall_01_0.mag_T;
    end
end

% angle of attack
if (config.use_airflow_measurement && ...
        topics.airflow_aoa.logged && ...
        params.cal_av_aoa_off.logged)
    % add the bias here since sysvector.airflow_aoa_0 already has the bias
    % subtracted and this function returns the angles with a bias
    aoa = sysvector.airflow_aoa_0.aoa_rad + deg2rad(paramvector.cal_av_aoa_off.Data);
else
    % print warnings if requested config cannot be met
    if (config.use_airflow_measurement && ...
            ~topics.airflow_aoa.logged)
        disp('WARNING: aoa airflow angles not logged, falling back to converting the raw measurements')
    end
    
    if (config.use_cal_av_params && ~aoa_params_logged)
        disp('WARNING: cal_av_aoa params not logged, falling back to using the manually set config')
    end
    
    % convert the data
    if aoa_hall_logged
        aoa = timeseries(deg2rad(1e-7 * aoa_rev * ...
            (aoa_p0 + ...
            aoa_p1 .* aoa_hall_data.Data + ...
            aoa_p2 .* aoa_hall_data.Data .* aoa_hall_data.Data + ...
            aoa_p3 .* aoa_hall_data.Data .* aoa_hall_data.Data .* aoa_hall_data.Data)), ...
            aoa_hall_data.Time);
    else
        aoa = timeseries([0.0, 0.0]', [config.st_time, config.end_time]);
        disp('WARNING: aoa hall not logged, setting aoa to 0')
        aoa_logged = false;
    end
end

% slip
if (config.use_airflow_measurement && ...
        topics.airflow_slip.logged && ...
        params.cal_av_slip_off.logged)
    % add the bias here since sysvector.airflow_slip_0 already has the bias
    % subtracted and this function returns the angles with a bias
    slip = sysvector.airflow_slip_0.slip_rad + deg2rad(paramvector.cal_av_slip_off.Data);
else
    % print warnings if requested config cannot be met
    if (config.use_airflow_measurement && ...
            ~topics.airflow_slip.logged)
        disp('WARNING: slip airflow angles not logged, falling back to converting the raw measurements')
    end
    
    if (config.use_cal_av_params && ~slip_params_logged)
        disp('WARNING: cal_av_slip params not logged, falling back to using the manually set config')
    end
    
    % convert the data
    if slip_hall_logged
        slip = timeseries(deg2rad(1e-7 * slip_rev * ...
            (slip_p0 + ...
            slip_p1 .* slip_hall_data.Data + ...
            slip_p2 .* slip_hall_data.Data .* slip_hall_data.Data + ...
            slip_p3 .* slip_hall_data.Data .* slip_hall_data.Data .* slip_hall_data.Data)), ...
            slip_hall_data.Time);
    else
        slip = timeseries([0.0, 0.0]', [config.st_time, config.end_time]);
        disp('WARNING: slip hall not logged, setting slip to 0')
        slip_logged = false;
    end
end
