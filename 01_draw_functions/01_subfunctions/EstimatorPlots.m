%% ************************************************************************
%  EstimatorPlots
%  ************************************************************************
%  Display the estimator data from the log file. Assumes that
%  successfully estimator data was logged.

function EstimatorPlots(sysvector, topics)
if topics.vehicle_attitude.logged
    %Attitude Estimate
    fig1 = figure();
    fig1.Name = 'Estimated Attitude';
    [pitch, roll, yaw] = ...
        QuaternionToEuler(sysvector.vehicle_attitude_0.q_0, sysvector.vehicle_attitude_0.q_1,...
        sysvector.vehicle_attitude_0.q_2, sysvector.vehicle_attitude_0.q_3);

    hold on;
    plot(roll.Time, roll.Data.*180./pi);
    plot(pitch.Time, pitch.Data.*180./pi);
    plot(yaw.Time, yaw.Data.*180./pi);
    hold off;
    title('Estimated attitude [deg]');
    legend('roll','pitch','yaw');
    grid on;
    dcm_obj = datacursormode(fig1);
    set(dcm_obj,'UpdateFcn',@HighPrecisionTooltipCallback);
end

ekf2_states_found = false;
if topics.estimator_states.logged
    estimator_data = sysvector.estimator_states_0;
    ekf2_states_found = true;

elseif topics.estimator_status.logged
    if isfield(sysvector.estimator_status_0, 'states_0')
        estimator_data = sysvector.estimator_status_0;
        ekf2_states_found = true;
    end
end

if ekf2_states_found
    % States
    fig2 = figure();
    fig2.Name = 'Estimated States';
    state(1) = subplot(3,3,1);
    hold on;
    plot(estimator_data.states_7.Time,estimator_data.states_7.Data);
    plot(estimator_data.states_8.Time,estimator_data.states_8.Data);
    plot(estimator_data.states_9.Time,estimator_data.states_9.Data);
    hold off;
    title('Position state (NED frame) [m]');
    state(2) = subplot(3,3,2);
    hold on;
    plot(estimator_data.states_0.Time,estimator_data.states_0.Data);
    plot(estimator_data.states_1.Time,estimator_data.states_1.Data);
    plot(estimator_data.states_2.Time,estimator_data.states_2.Data);
    plot(estimator_data.states_3.Time,estimator_data.states_3.Data);
    hold off;
    title('Angular states');
    state(3) = subplot(3,3,3);
    hold on;
    plot(estimator_data.states_4.Time,estimator_data.states_4.Data);
    plot(estimator_data.states_5.Time,estimator_data.states_5.Data);
    plot(estimator_data.states_6.Time,estimator_data.states_6.Data);
    hold off;
    title('Velocity states (NED frame) [m/s]');
    state(4) = subplot(3,3,4);
    hold on;
    plot(estimator_data.states_10.Time,estimator_data.states_10.Data);
    plot(estimator_data.states_11.Time,estimator_data.states_11.Data);
    plot(estimator_data.states_12.Time,estimator_data.states_12.Data);
    hold off;
    title('Gyroscope bias states (body frame) [rad/s]');
    state(5) = subplot(3,3,5);
    hold on;
    plot(estimator_data.states_13.Time,estimator_data.states_13.Data);
    plot(estimator_data.states_14.Time,estimator_data.states_14.Data);
    plot(estimator_data.states_15.Time,estimator_data.states_15.Data);
    hold off;
    title('Accelerometer bias states (body frame) [m/s^2]');
    state(6) = subplot(3,3,6);
    hold on;
    plot(estimator_data.states_16.Time,estimator_data.states_16.Data);
    plot(estimator_data.states_17.Time,estimator_data.states_17.Data);
    plot(estimator_data.states_18.Time,estimator_data.states_18.Data);
    hold off;
    title('Earth magnetic field states (NED frame) [gauss]');
    state(7) = subplot(3,3,7);
    hold on;
    plot(estimator_data.states_22.Time,estimator_data.states_22.Data);
    plot(estimator_data.states_23.Time,estimator_data.states_23.Data);
    if (isfield(estimator_data, 'states_24'))
        plot(estimator_data.states_24.Time,estimator_data.states_24.Data);
    end
    hold off;
    title('Wind states (NED frame) [m/s]');
    state(8) = subplot(3,3,9);
    hold on;
    plot(estimator_data.states_19.Time,estimator_data.states_19.Data);
    plot(estimator_data.states_20.Time,estimator_data.states_20.Data);
    plot(estimator_data.states_21.Time,estimator_data.states_21.Data);
    hold off;
    title('Magnetometer bias states (body frame) [gauss]');

    linkaxes([state(1) state(2) state(3) state(4) state(5) state(6) state(7) state(8)],'x');
    set(state(:),'XGrid','on','YGrid','on','ZGrid','on');
    dcm_obj = datacursormode(fig2);
    set(dcm_obj,'UpdateFcn',@HighPrecisionTooltipCallback);

    % Covariances
    fig3 = figure();
    fig3.Name = 'Estimated Covariances';
    state(1) = subplot(3,3,1);
    hold on;
    plot(estimator_data.covariances_7.Time,estimator_data.covariances_7.Data);
    plot(estimator_data.covariances_8.Time,estimator_data.covariances_8.Data);
    plot(estimator_data.covariances_9.Time,estimator_data.covariances_9.Data);
    hold off;
    title('Position covariances');
    state(2) = subplot(3,3,2);
    hold on;
    plot(estimator_data.covariances_0.Time,estimator_data.covariances_0.Data);
    plot(estimator_data.covariances_1.Time,estimator_data.covariances_1.Data);
    plot(estimator_data.covariances_2.Time,estimator_data.covariances_2.Data);
    plot(estimator_data.covariances_3.Time,estimator_data.covariances_3.Data);
    hold off;
    title('Angular covariances');
    state(3) = subplot(3,3,3);
    hold on;
    plot(estimator_data.covariances_4.Time,estimator_data.covariances_4.Data);
    plot(estimator_data.covariances_5.Time,estimator_data.covariances_5.Data);
    plot(estimator_data.covariances_6.Time,estimator_data.covariances_6.Data);
    hold off;
    title('Velocity covariances');
    state(4) = subplot(3,3,4);
    hold on;
    plot(estimator_data.covariances_10.Time,estimator_data.covariances_10.Data);
    plot(estimator_data.covariances_11.Time,estimator_data.covariances_11.Data);
    plot(estimator_data.covariances_12.Time,estimator_data.covariances_12.Data);
    hold off;
    title('Gyroscope bias covariances');
    state(5) = subplot(3,3,5);
    hold on;
    plot(estimator_data.covariances_13.Time,estimator_data.covariances_13.Data);
    plot(estimator_data.covariances_14.Time,estimator_data.covariances_14.Data);
    plot(estimator_data.covariances_15.Time,estimator_data.covariances_15.Data);
    hold off;
    title('Accelerometer bias covariances');
    state(6) = subplot(3,3,6);
    hold on;
    plot(estimator_data.covariances_16.Time,estimator_data.covariances_16.Data);
    plot(estimator_data.covariances_17.Time,estimator_data.covariances_17.Data);
    plot(estimator_data.covariances_18.Time,estimator_data.covariances_18.Data);
    hold off;
    title('Earth magnetic field covariances');
    state(7) = subplot(3,3,7);
    hold on;
    plot(estimator_data.covariances_22.Time,estimator_data.covariances_22.Data);
    plot(estimator_data.covariances_23.Time,estimator_data.covariances_23.Data);
    if (isfield(estimator_data, 'covariances_24'))
        plot(estimator_data.covariances_24.Time,estimator_data.covariances_24.Data);
    end
    hold off;
    title('Wind covariances');
    state(8) = subplot(3,3,9);
    hold on;
    plot(estimator_data.covariances_19.Time,estimator_data.covariances_19.Data);
    plot(estimator_data.covariances_20.Time,estimator_data.covariances_20.Data);
    plot(estimator_data.covariances_21.Time,estimator_data.covariances_21.Data);
    hold off;
    title('Magnetometer bias covariances');

    linkaxes([state(1) state(2) state(3) state(4) state(5) state(6) state(7) state(8)],'x');
    set(state(:),'XGrid','on','YGrid','on','ZGrid','on');
    dcm_obj = datacursormode(fig3);
    set(dcm_obj,'UpdateFcn',@HighPrecisionTooltipCallback);
end

% plot the relative timestamps
if topics.ekf2_timestamps.logged
    fig4 = figure();
    fig4.Name = 'Estimator Relative Timestamps';
    timestamp_rel(1) = subplot(3,2,1);
    plot(sysvector.ekf2_timestamps_0.airspeed_timestamp_rel.Time,...
        sysvector.ekf2_timestamps_0.airspeed_timestamp_rel.Data*0.1);
    title('EKF2 Airspeed Relative Timestamp [ms]');
    
    timestamp_rel(2) = subplot(3,2,2);
    plot(sysvector.ekf2_timestamps_0.distance_sensor_timestamp_rel.Time,...
        sysvector.ekf2_timestamps_0.distance_sensor_timestamp_rel.Data*0.1);
    title('EKF2 Distance Sensor Relative Timestamp [ms]');
    
    timestamp_rel(3) = subplot(3,2,3);
    plot(sysvector.ekf2_timestamps_0.gps_timestamp_rel.Time,...
        sysvector.ekf2_timestamps_0.gps_timestamp_rel.Data*0.1);
    title('EKF2 GPS Relative Timestamp [ms]');
    
    timestamp_rel(4) = subplot(3,2,4);
    plot(sysvector.ekf2_timestamps_0.optical_flow_timestamp_rel.Time,...
        sysvector.ekf2_timestamps_0.optical_flow_timestamp_rel.Data*0.1);
    title('EKF2 Optical Flow Relative Timestamp [ms]');
    
    timestamp_rel(5) = subplot(3,2,5);
    plot(sysvector.ekf2_timestamps_0.vision_attitude_timestamp_rel.Time,...
        sysvector.ekf2_timestamps_0.vision_attitude_timestamp_rel.Data*0.1);
    title('EKF2 Vision Attitude Relative Timestamp [ms]');
    
    timestamp_rel(6) = subplot(3,2,6);
    plot(sysvector.ekf2_timestamps_0.vision_position_timestamp_rel.Time,...
        sysvector.ekf2_timestamps_0.vision_position_timestamp_rel.Data*0.1);
    title('EKF2 Vision Position Relative Timestamp [ms]');

    linkaxes([timestamp_rel(1) timestamp_rel(2) timestamp_rel(3) ...
        timestamp_rel(4) timestamp_rel(5) timestamp_rel(6)],'x');
    set(timestamp_rel(:), 'XGrid','on','YGrid','on','ZGrid','on');
    dcm_obj = datacursormode(fig4);
    set(dcm_obj,'UpdateFcn',@HighPrecisionTooltipCallback);
end