function AirflowAnglePlots(sysvector, topics)
% Display the airflow angle data.

if (topics.estimator_status.logged)
    va_n = sysvector.estimator_status_0.states_4.Data - sysvector.estimator_status_0.states_22.Data;
    va_e = sysvector.estimator_status_0.states_5.Data - sysvector.estimator_status_0.states_23.Data;
    if isfield(sysvector.estimator_status_0,'states_24')
        % check if we have wind z estimates
        va_d = sysvector.estimator_status_0.states_6.Data - sysvector.estimator_status_0.states_24.Data;
    else
        va_d = sysvector.estimator_status_0.states_6.Data; % assume zero down wind
    end
    Hbi = quat2dcm([sysvector.estimator_status_0.states_0.Data, ...
        sysvector.estimator_status_0.states_1.Data, ...
        sysvector.estimator_status_0.states_2.Data, ...
        sysvector.estimator_status_0.states_3.Data]);
    len_t = length(sysvector.estimator_status_0.states_0.Time);
    vb = zeros(len_t,3);
    for ii = 1:len_t
        vb(ii,:) = Hbi(:,:,ii) * [va_n(ii); va_e(ii); va_d(ii)];
    end
end

figure('color', 'w', 'name', 'Airflow Angles'); 

airflow(1) = subplot(4,1,1); hold on; grid on; box on;
if (topics.airflow_aoa.logged)
    ax_meas = plot(sysvector.airflow_aoa_0.aoa_rad.Time, ...
        rad2deg(sysvector.airflow_aoa_0.aoa_rad.Data));
    str_meas = {'Measurement'};
else
    ax_meas = [];
    str_meas = {};
end
if (topics.estimator_status.logged)
    ax_est = plot(sysvector.estimator_status_0.states_0.Time, ...
        atan2d(vb(:,3), vb(:,1)));
    str_est = {'Estimate'};
else
    ax_est = [];
    str_est = {};
end
ylabel('Angle of attack [deg]');
legend([ax_meas,ax_est], [str_meas,str_est]);

airflow(2) = subplot(4,1,2); hold on; grid on; box on;
if (topics.airflow_slip.logged)
    ax_meas = plot(sysvector.airflow_slip_0.slip_rad.Time, ...
        rad2deg(sysvector.airflow_slip_0.slip_rad.Data));
    str_meas = {'Measurement'};
else
    ax_meas = [];
    str_meas = {};
end
if (topics.estimator_status.logged)
    ax_est = plot(sysvector.estimator_status_0.states_0.Time, ...
        atan2d(vb(:,2), vb(:,1)));
    str_est = {'Estimate'};
else
    ax_est = [];
    str_est = {};
end
ylabel('slip [deg]');
legend([ax_meas,ax_est], [str_meas,str_est]);

airflow(3) = subplot(4,1,3); hold on; grid on; box on;
if (topics.airflow_aoa.logged)
    ax_aoa = plot(sysvector.airflow_aoa_0.valid.Time, sysvector.airflow_aoa_0.valid.Data);
    str_aoa = {'AoA'};
else
    ax_aoa = [];
    str_aoa = {};
end
if (topics.airflow_slip.logged)
    ax_slip = plot(sysvector.airflow_slip_0.valid.Time, sysvector.airflow_slip_0.valid.Data);
    str_slip = {'Slip'};
else
    ax_slip = [];
    str_slip = {};
end
legend([ax_aoa,ax_slip],[str_aoa,str_slip]);
ylabel('Valid Meas. [~]');

airflow(4) = subplot(4,1,4); hold on; grid on; box on;
if (topics.sensor_hall.logged)
    ax_aoa = plot(sysvector.sensor_hall_0.mag_T.Time, sysvector.sensor_hall_0.mag_T.Data);
    str_aoa = {'AoA'};
else
    ax_aoa = [];
    str_aoa = {};
end
if (topics.sensor_hall_01.logged)
    ax_slip = plot(sysvector.sensor_hall_01_0.mag_T.Time, sysvector.sensor_hall_01_0.mag_T.Data);
    str_slip = {'Slip'};
else
    ax_slip = [];
    str_slip = {};
end
legend([ax_aoa,ax_slip],[str_aoa,str_slip]);
ylabel('Hall Effect [mT]');

linkaxes(airflow(:),'x');
set(airflow(:),'XGrid','on','YGrid','on','ZGrid','on');
end

