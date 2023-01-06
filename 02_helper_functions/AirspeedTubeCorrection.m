function [dp_raw, dp_filtered, airspeed_indicated, airspeed_true,...
    airspeed_true_unfiltered] = AirspeedTubeCorrection(sysvector, D, L,...
    dpSensor, massflow)
% Correct the airspeed and differential pressure based on the pressure loss
% in the pitot tube

% synchronise baro data
baro_pressure = sysvector.sensor_baro_0.pressure;
if (sysvector.sensor_baro_0.pressure.Time(1) > sysvector.differential_pressure_0.temperature.Time(1))
    baro_pressure = addsample(baro_pressure,...
        'Data', sysvector.sensor_baro_0.pressure.Data(1),...
        'Time', sysvector.differential_pressure_0.temperature.Time(1));
end
if (sysvector.sensor_baro_0.pressure.Time(end) < sysvector.differential_pressure_0.temperature.Time(end))
    baro_pressure = addsample(baro_pressure,...
        'Data', sysvector.sensor_baro_0.pressure.Data(end),...
        'Time', sysvector.differential_pressure_0.temperature.Time(end));
end
baro_pressure.DataInfo.Interpolation = tsdata.interpolation('zoh');
baro_pressure = resample(baro_pressure, sysvector.differential_pressure_0.temperature.Time);

% convert absolute pressure from milibar to pascal
baro_pressure.Data = baro_pressure.Data * 100;

if isfield(sysvector.differential_pressure_0, 'differential_pressure_raw_pa')
    data_dp_raw = sysvector.differential_pressure_0.differential_pressure_raw_pa;
    data_dp_filtered = sysvector.differential_pressure_0.differential_pressure_filtered_pa;
else
    data_dp_raw = sysvector.differential_pressure_0.differential_pressure_pa;
    data_dp_filtered = sysvector.differential_pressure_0.differential_pressure_pa;
end

eps_raw = CalculateDifferentialPressureCorrectionFactor(...
    data_dp_raw.Data,...
    sysvector.differential_pressure_0.temperature.Data,...
    baro_pressure.Data, D, L, dpSensor, massflow);
eps_filtered = CalculateDifferentialPressureCorrectionFactor(...
    data_dp_filtered.Data,...
    sysvector.differential_pressure_0.temperature.Data,...
    baro_pressure.Data, D, L, dpSensor, massflow);

% correct dp
dp_raw = timeseries(data_dp_raw.Data./(1 + eps_raw), ...
    data_dp_raw.Time);
dp_filtered = timeseries(data_dp_filtered.Data./(1 + eps_filtered),...
    data_dp_filtered.Time);

% compute indicated airspeed
airspeed_indicated = timeseries(sqrt(2.0*abs(dp_filtered.Data) / 1.225), dp_filtered.Time);
airspeed_indicated.Data(dp_filtered.Data < 0.0) = 0.0;
airspeed_indicated_unfiltered = timeseries(sqrt(2.0*abs(dp_raw.Data) / 1.225), dp_raw.Time);
airspeed_indicated_unfiltered.Data(dp_raw.Data < 0.0) = 0.0;

% compute the true airspeed from the indicated airspeed
air_density = (baro_pressure.Data)./...
    (287.1 .* (273.15 + sysvector.differential_pressure_0.temperature.Data));
airspeed_true = ...
    timeseries(airspeed_indicated.Data .* sqrt(1.225 ./ air_density), airspeed_indicated.Time);
airspeed_true_unfiltered = ...
    timeseries(airspeed_indicated_unfiltered.Data .* sqrt(1.225 ./ air_density), airspeed_indicated_unfiltered.Time);

% resample the airspeeds
if (airspeed_indicated.Time(1) > sysvector.airspeed_0.indicated_airspeed_m_s.Time(1))
    airspeed_indicated = airspeed_indicated.addsample('Data', airspeed_indicated.Data(1),...
        'Time', sysvector.airspeed_0.indicated_airspeed_m_s.Time(1));
    airspeed_true = airspeed_true.addsample('Data', airspeed_true.Data(1),...
        'Time', sysvector.airspeed_0.indicated_airspeed_m_s.Time(1));
    airspeed_true_unfiltered = airspeed_true_unfiltered.addsample('Data', airspeed_true_unfiltered.Data(1),...
        'Time', sysvector.airspeed_0.indicated_airspeed_m_s.Time(1));
end
if (airspeed_indicated.Time(end) < sysvector.airspeed_0.indicated_airspeed_m_s.Time(end))
    airspeed_indicated = airspeed_indicated.addsample('Data', airspeed_indicated.Data(end),...
        'Time', sysvector.airspeed_0.indicated_airspeed_m_s.Time(end));
    airspeed_true = airspeed_true.addsample('Data', airspeed_true.Data(end),...
        'Time', sysvector.airspeed_0.indicated_airspeed_m_s.Time(end));
    airspeed_true_unfiltered = airspeed_true_unfiltered.addsample('Data', airspeed_true_unfiltered.Data(end),...
        'Time', sysvector.airspeed_0.indicated_airspeed_m_s.Time(end));
end
airspeed_indicated = resample(airspeed_indicated, sysvector.airspeed_0.indicated_airspeed_m_s.Time);
airspeed_true = resample(airspeed_true, sysvector.airspeed_0.indicated_airspeed_m_s.Time);
airspeed_true_unfiltered = resample(airspeed_true_unfiltered, sysvector.airspeed_0.indicated_airspeed_m_s.Time);
