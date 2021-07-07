function AirflowAnglePlots(sysvector, topics)
% Display the airflow angle data.

estimator_data_available = false;
if topics.estimator_states.logged
    estimator_data = sysvector.estimator_states_0;
    estimator_data_available = true;
elseif topics.estimator_status.logged
    if isfield(sysvector.estimator_status_0, 'states_4')
        estimator_data = sysvector.estimator_status_0;
        estimator_data_available = true;
    end
end

if estimator_data_available
    va_n = estimator_data.states_4.Data - estimator_data.states_22.Data;
    va_e = estimator_data.states_5.Data - estimator_data.states_23.Data;
    if isfield(estimator_data,'states_24')
        % check if we have wind z estimates
        va_d = estimator_data.states_6.Data - estimator_data.states_24.Data;
    else
        va_d = estimator_data.states_6.Data; % assume zero down wind
    end
    Hbi = quat2dcm([estimator_data.states_0.Data, ...
        estimator_data.states_1.Data, ...
        estimator_data.states_2.Data, ...
        estimator_data.states_3.Data]);
    len_t = length(estimator_data.states_0.Time);
    vb = zeros(len_t,3);
    for ii = 1:len_t
        vb(ii,:) = Hbi(:,:,ii) * [va_n(ii); va_e(ii); va_d(ii)];
    end
end

figure('color', 'w', 'name', 'Airflow Angles'); 

airflow(1) = subplot(4,1,1); hold on; grid on; box on;
if topics.airflow_aoa.logged
    ax_meas = plot(sysvector.airflow_aoa_0.aoa_rad.Time, ...
        rad2deg(sysvector.airflow_aoa_0.aoa_rad.Data));
    str_meas = {'Measurement'};
else
    ax_meas = [];
    str_meas = {};
end
if estimator_data_available
    ax_est = plot(estimator_data.states_0.Time, ...
        atan2d(vb(:,3), vb(:,1)));
    str_est = {'Estimate'};
else
    ax_est = [];
    str_est = {};
end
ylabel('Angle of attack [deg]');
legend([ax_meas,ax_est], [str_meas,str_est]);

airflow(2) = subplot(4,1,2); hold on; grid on; box on;
if topics.airflow_slip.logged
    ax_meas = plot(sysvector.airflow_slip_0.slip_rad.Time, ...
        rad2deg(sysvector.airflow_slip_0.slip_rad.Data));
    str_meas = {'Measurement'};
else
    ax_meas = [];
    str_meas = {};
end
if estimator_data_available
    ax_est = plot(estimator_data.states_0.Time, ...
        atan2d(vb(:,2), vb(:,1)));
    str_est = {'Estimate'};
else
    ax_est = [];
    str_est = {};
end
ylabel('Sideslip [deg]');
legend([ax_meas,ax_est], [str_meas,str_est]);

airflow(3) = subplot(4,1,3); hold on; grid on; box on;
if topics.airflow_aoa.logged
    ax_aoa = plot(sysvector.airflow_aoa_0.valid.Time, sysvector.airflow_aoa_0.valid.Data);
    str_aoa = {'AoA'};
else
    ax_aoa = [];
    str_aoa = {};
end
if topics.airflow_slip.logged
    ax_aos = plot(sysvector.airflow_slip_0.valid.Time, sysvector.airflow_slip_0.valid.Data);
    str_aos = {'Slip'};
else
    ax_aos = [];
    str_aos = {};
end
legend([ax_aoa,ax_aos],[str_aoa,str_aos]);
ylabel('Valid Meas. [~]');

airflow(4) = subplot(4,1,4); hold on; grid on; box on;
if topics.sensor_hall.logged
    ax_hall = [];
    str_hall = {};
    for i = 0:topics.sensor_hall.num_instances-1
        data = getfield(sysvector, 'sensor_hall_' + string(i));
        ax_hall(end+1) = plot(data.mag_t.Time, data.mag_t.Data);
        str_hall(end+1) = {'hall ' + string(i)};
    end
else
    ax_hall = [];
    str_hall = {};
end
if topics.sensor_hall_01.logged
    ax_hall_01 = plot(sysvector.sensor_hall_01_0.mag_t.Time, sysvector.sensor_hall_01_0.mag_t.Data);
    str_hall_01 = {'hall 01'};
else
    ax_hall_01 = [];
    str_hall_01 = {};
end
legend([ax_hall,ax_hall_01],[str_hall,str_hall_01]);
ylabel('Hall Effect [mT]');

linkaxes(airflow(:),'x');
set(airflow(:),'XGrid','on','YGrid','on','ZGrid','on');
end

