%% ************************************************************************
%  DisplayDiffPressLogData
%  ************************************************************************
%  Display the differential pressure data from the log file. Assumes that
%  successfully differential pressure data was logged.

function DisplayDiffPressLogData(sysvector, topics, plotvector)
% correct differential pressure and airspeed if required
if plotvector.doPressureCorrection
    [dp_raw, dp_filtered, airspeed_indicated, airspeed_true,...
        airspeed_true_unfiltered] = AirspeedTubeCorrection(sysvector, ...
        plotvector.pressureCorrectionD, plotvector.pressureCorrectionL);
    figure('Name', 'Pressure Tube Correction');
    hold on;
    plot(airspeed_indicated.Time, airspeed_indicated.Data);
    plot(airspeed_true.Time, airspeed_true.Data);
    plot(sysvector('airspeed.indicated_airspeed').Time, sysvector('airspeed.indicated_airspeed').Data);
    plot(sysvector('airspeed.true_airspeed').Time, sysvector('airspeed.true_airspeed').Data);
    legend('Indicated Airspeed (IAS) corrected','True Airspeed (TAS) corrected', ...
        'Indicated Airspeed (IAS) uncorrected','True Airspeed (TAS) uncorrected');
    title('Airspeed (Tube Correction) [m/s]');
    hold off;
else
    dp_raw = sysvector('differential_pressure.differential_pressure_raw');
    dp_filtered = sysvector('differential_pressure.differential_pressure_filtered');
    airspeed_indicated = sysvector('airspeed.indicated_airspeed');
    airspeed_true = sysvector('airspeed.true_airspeed');
    airspeed_true_unfiltered = sysvector('airspeed.true_airspeed_unfiltered');
end


figure('Name', 'Differential Pressure Data');
raw_baro(1) = subplot(4,1,1);
hold on;
plot(dp_raw.Time, dp_raw.Data);
plot(dp_filtered.Time, dp_filtered.Data);
hold off;
legend('raw','filtered');
title('dbaro [Pa]');

[v_gps] = CalcGPSHorizontalVelocity(sysvector, topics);
    
raw_baro(2) = subplot(4,1,2);
hold on;
plot(airspeed_true_unfiltered.Time, airspeed_true_unfiltered.Data);
plot(airspeed_true.Time, airspeed_true.Data);
plot(v_gps.Time,v_gps.Data);
hold off;
title('differential barometer and GPS velocity [m/s]');    
legend('dbaro raw', 'dbaro filtered','GPS');

% compute the velocity difference at 20 Hz
v_diff = TimeseriesSubtraction(airspeed_true, v_gps, 0.05);

raw_baro(3) = subplot(4,1,3); 
plot(v_diff.Time,v_diff.Data);
title('diff dbaro and GPS velocity [m/s]');

raw_baro(4) = subplot(4,1,4);
hold on;
plot(airspeed_indicated.Time, airspeed_indicated.Data);
plot(airspeed_true.Time, airspeed_true.Data);
% TODO add here gps airspeed
hold off;
legend('Indicated Airspeed (IAS)','True Airspeed (TAS)');
title('Airspeed [m/s]');

linkaxes([raw_baro(1) raw_baro(2) raw_baro(3) raw_baro(4)],'x');
set(raw_baro(:),'XGrid','on','YGrid','on','ZGrid','on');   
end