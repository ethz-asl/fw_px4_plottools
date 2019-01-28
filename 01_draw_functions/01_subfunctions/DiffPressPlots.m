%% ************************************************************************
%  DiffPressPlots
%  ************************************************************************
%  Display the differential pressure data from the log file. Assumes that
%  successfully differential pressure data was logged.

function DiffPressPlots(sysvector, topics, plotvector)
% correct differential pressure and airspeed if required
if plotvector.doPressureCorrection && topics.sensor_baro.logged
    [dp_raw, dp_filtered, airspeed_indicated, airspeed_true,...
        airspeed_true_unfiltered] = AirspeedTubeCorrection(sysvector,...
        plotvector.pressureCorrectionD, plotvector.pressureCorrectionL,...
        plotvector.pressureCorrectionDPSensor, plotvector.pressureCorrectionMassflow);
    fig1 = figure();
    fig1.Name = 'Pressure Tube Correction';
    hold on;
    plot(airspeed_indicated.Time, airspeed_indicated.Data);
    plot(airspeed_true.Time, airspeed_true.Data);
    plot(sysvector.airspeed_0.indicated_airspeed_m_s.Time, sysvector.airspeed_0.indicated_airspeed_m_s.Data);
    plot(sysvector.airspeed_0.true_airspeed_m_s.Time, sysvector.airspeed_0.true_airspeed_m_s.Data);
    legend('Indicated Airspeed (IAS) corrected','True Airspeed (TAS) corrected', ...
        'Indicated Airspeed (IAS) uncorrected','True Airspeed (TAS) uncorrected');
    title('Airspeed (Tube Correction) [m/s]');
    hold off;
    dcm_obj = datacursormode(fig1);
    set(dcm_obj,'UpdateFcn',@HighPrecisionTooltipCallback);


    fig2 = figure();
    fig2.Name = 'Pressure Tube Correction Difference';
    diff_true_airspeed = TimeseriesSubtraction(airspeed_true, sysvector.airspeed_0.true_airspeed_m_s, 0.05);
    diff_indicated_airspeed = TimeseriesSubtraction(airspeed_indicated, sysvector.airspeed_0.indicated_airspeed_m_s, 0.05);
    hold on;
    plot(diff_true_airspeed.Time, diff_true_airspeed.Data);
    plot(diff_indicated_airspeed.Time, diff_indicated_airspeed.Data);
    legend('Difference for True Airspeed (TAS)','Difference for Indicated Airspeed (IAS)');
    title('Difference between corrected and uncorrected airspeed [m/s]');
    hold off;
    dcm_obj = datacursormode(fig2);
    set(dcm_obj,'UpdateFcn',@HighPrecisionTooltipCallback);

    fig3 = figure();
    fig3.Name = 'Pressure Tube Correction Raw Data';
    raw_corr(1) = subplot(3,1,1);
    plot(sysvector.sensor_baro_0.pressure.Time, sysvector.sensor_baro_0.pressure.Data);
    title('Raw ambient pressure [mbar]');

    raw_corr(2) = subplot(3,1,2);
    plot(sysvector.differential_pressure_0.differential_pressure_raw_pa.Time,...
        sysvector.differential_pressure_0.differential_pressure_raw_pa.Data);
    title('Raw differential pressure [pa]');

    raw_corr(3) = subplot(3,1,3);
    plot(sysvector.differential_pressure_0.temperature.Time,...
        sysvector.differential_pressure_0.temperature.Data);
    title('Raw ambient temperature [C]');
    linkaxes([raw_corr(1) raw_corr(2) raw_corr(3)],'x');
    set(raw_corr(:),'XGrid','on','YGrid','on','ZGrid','on');
    dcm_obj = datacursormode(fig3);
    set(dcm_obj,'UpdateFcn',@HighPrecisionTooltipCallback);
else
    dp_raw = sysvector.differential_pressure_0.differential_pressure_raw_pa;
    dp_filtered = sysvector.differential_pressure_0.differential_pressure_filtered_pa;
    airspeed_indicated = sysvector.airspeed_0.indicated_airspeed_m_s;
    airspeed_true = sysvector.airspeed_0.true_airspeed_m_s;
    airspeed_true_unfiltered = sysvector.airspeed_0.true_airspeed_unfiltered_m_s;
end

if plotvector.doPressureCorrection && (~topics.sensor_baro.logged)
   disp('Could not execute the pressure correction as no baro data was logged');
end

fig4 = figure();
fig4.Name = 'Differential Pressure Data';
raw_baro(1) = subplot(4,1,1);
hold on;
plot(dp_raw.Time, dp_raw.Data);
plot(dp_filtered.Time, dp_filtered.Data);
hold off;
legend('raw','filtered');
title('dbaro [Pa]');

if topics.vehicle_gps_position.logged
    [v_gps] = CalcGPSHorizontalVelocity(sysvector, topics);
end

raw_baro(2) = subplot(4,1,2);
hold on;
plot(airspeed_true_unfiltered.Time, airspeed_true_unfiltered.Data);
plot(airspeed_true.Time, airspeed_true.Data);
if topics.vehicle_gps_position.logged
    plot(v_gps.Time,v_gps.Data);
end
hold off;
title('True Airspeed (from dBaro) and GPS velocity [m/s]');    
legend('TAS raw', 'TAS filtered+corrected','GPS');

raw_baro(3) = subplot(4,1,3);
if topics.vehicle_gps_position.logged
    % compute the velocity difference at 20 Hz
    v_diff = TimeseriesSubtraction(airspeed_true, v_gps, 0.05);

    plot(v_diff.Time,v_diff.Data);
end
title('difference dbaro and GPS velocity [m/s]');


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
dcm_obj = datacursormode(fig4);
set(dcm_obj,'UpdateFcn',@HighPrecisionTooltipCallback);
end