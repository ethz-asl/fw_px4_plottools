function AirflowAnglePlots(sysvector, topics)
% Display the airflow angle data.

figure('color', 'w', 'name', 'Airflow Angles'); 

airflow(1) = subplot(4,1,1); hold on; grid on; box on;
if (topics.airflow_aoa.logged)
    plot(sysvector.airflow_aoa_0.aoa_rad.Time, ...
        rad2deg(sysvector.airflow_aoa_0.aoa_rad.Data));
end
ylabel('Angle of attack [deg]');

airflow(2) = subplot(4,1,2); hold on; grid on; box on;
if (topics.airflow_aos.logged)
    plot(sysvector.airflow_aos_0.aos_rad.Time, ...
        rad2deg(sysvector.airflow_aos_0.aos_rad.Data));
end
ylabel('Sideslip [deg]');

airflow(3) = subplot(4,1,3); hold on; grid on; box on;
if (topics.airflow_aoa.logged)
    ax_aoa = plot(sysvector.airflow_aoa_0.valid.Time, sysvector.airflow_aoa_0.valid.Data);
    str_aoa = {'AoA'};
else
    ax_aoa = [];
    str_aoa = {};
end
if (topics.airflow_aos.logged)
    ax_aos = plot(sysvector.airflow_aos_0.valid.Time, sysvector.airflow_aos_0.valid.Data);
    str_aos = {'Slip'};
else
    ax_aos = [];
    str_aos = {};
end
legend([ax_aoa,ax_aos],[str_aoa,str_aos]);
ylabel('Valid Meas. [~]');

airflow(4) = subplot(4,1,4); hold on; grid on; box on;
if (topics.airflow_aoa.logged)
    ax_aoa = plot(sysvector.sensor_hall_0.mag_T.Time, sysvector.sensor_hall_0.mag_T.Data);
    str_aoa = {'AoA'};
else
    ax_aoa = [];
    str_aoa = {};
end
if (topics.airflow_aos.logged)
    ax_aos = plot(sysvector.sensor_hall_01_0.mag_T.Time, sysvector.sensor_hall_01_0.mag_T.Data);
    str_aos = {'Slip'};
else
    ax_aos = [];
    str_aos = {};
end
legend([ax_aoa,ax_aos],[str_aoa,str_aos]);
ylabel('Hall Effect [mT]');

linkaxes(airflow(:),'x');
set(airflow(:),'XGrid','on','YGrid','on','ZGrid','on');
end

