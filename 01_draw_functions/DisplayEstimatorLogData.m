%% ************************************************************************
%  DisplayEstimatorLogData
%  ************************************************************************
%  Display the estimator data from the log file. Assumes that
%  successfully estimator data was logged.

function DisplayEstimatorLogData(sysvector, topics)
if topics.vehicle_attitude.logged
    %Attitude Estimate
    figure('Name', 'Estimated Attitude');
    [pitch, roll, yaw] = ...
        QuaternionToEuler(sysvector('vehicle_attitude.q_0'), sysvector('vehicle_attitude.q_1'),...
        sysvector('vehicle_attitude.q_2'), sysvector('vehicle_attitude.q_3'));

    hold on;
    plot(roll.Time, roll.Data.*180./pi);
    plot(pitch.Time, pitch.Data.*180./pi);
    plot(yaw.Time, yaw.Data.*180./pi);
    hold off;
    title('Estimated attitude [deg]');
    legend('roll','pitch','yaw');
    grid on;
end

if topics.estimator_status.logged
    % States
    figure('Name', 'Estimated States');
    state(1) = subplot(3,3,1);
    hold on;
    plot(sysvector('estimator_status.states_7').Time,sysvector('estimator_status.states_7').Data);
    plot(sysvector('estimator_status.states_8').Time,sysvector('estimator_status.states_8').Data);
    plot(sysvector('estimator_status.states_9').Time,sysvector('estimator_status.states_9').Data);
    hold off;
    title('Position state (NED frame) [m]');
    state(2) = subplot(3,3,2);
    hold on;
    plot(sysvector('estimator_status.states_0').Time,sysvector('estimator_status.states_0').Data);
    plot(sysvector('estimator_status.states_1').Time,sysvector('estimator_status.states_1').Data);
    plot(sysvector('estimator_status.states_2').Time,sysvector('estimator_status.states_2').Data);
    plot(sysvector('estimator_status.states_3').Time,sysvector('estimator_status.states_3').Data);
    hold off;
    title('Angular states');
    state(3) = subplot(3,3,3);
    hold on;
    plot(sysvector('estimator_status.states_4').Time,sysvector('estimator_status.states_4').Data);
    plot(sysvector('estimator_status.states_5').Time,sysvector('estimator_status.states_5').Data);
    plot(sysvector('estimator_status.states_6').Time,sysvector('estimator_status.states_6').Data);
    hold off;
    title('Velocity states (NED frame) [m/s]');
    state(4) = subplot(3,3,4);
    hold on;
    plot(sysvector('estimator_status.states_10').Time,sysvector('estimator_status.states_10').Data);
    plot(sysvector('estimator_status.states_11').Time,sysvector('estimator_status.states_11').Data);
    plot(sysvector('estimator_status.states_12').Time,sysvector('estimator_status.states_12').Data);
    hold off;
    title('Gyroscope bias states (body frame) [rad/s]');
    state(5) = subplot(3,3,5);
    hold on;
    plot(sysvector('estimator_status.states_13').Time,sysvector('estimator_status.states_13').Data);
    plot(sysvector('estimator_status.states_14').Time,sysvector('estimator_status.states_14').Data);
    plot(sysvector('estimator_status.states_15').Time,sysvector('estimator_status.states_15').Data);
    hold off;
    title('Accelerometer bias states (body frame) [m/s^2]');
    state(6) = subplot(3,3,6);
    hold on;
    plot(sysvector('estimator_status.states_16').Time,sysvector('estimator_status.states_16').Data);
    plot(sysvector('estimator_status.states_17').Time,sysvector('estimator_status.states_17').Data);
    plot(sysvector('estimator_status.states_18').Time,sysvector('estimator_status.states_18').Data);
    hold off;
    title('Earth magnetic field states (NED frame) [gauss]');
    state(7) = subplot(3,3,7);
    hold on;
    plot(sysvector('estimator_status.states_22').Time,sysvector('estimator_status.states_22').Data);
    plot(sysvector('estimator_status.states_23').Time,sysvector('estimator_status.states_23').Data);
    hold off;
    title('Wind states (NED frame) [m/s]');
    state(8) = subplot(3,3,9);
    hold on;
    plot(sysvector('estimator_status.states_19').Time,sysvector('estimator_status.states_19').Data);
    plot(sysvector('estimator_status.states_20').Time,sysvector('estimator_status.states_20').Data);
    plot(sysvector('estimator_status.states_21').Time,sysvector('estimator_status.states_21').Data);
    hold off;
    title('Magnetometer bias states (body frame) [gauss]');

    linkaxes([state(1) state(2) state(3) state(4) state(5) state(6) state(7) state(8)],'x');
    set(state(:),'XGrid','on','YGrid','on','ZGrid','on');

    % Covariances
    figure('Name', 'Estimated Covariances');
    state(1) = subplot(3,3,1);
    hold on;
    plot(sysvector('estimator_status.covar_7').Time,sysvector('estimator_status.covar_7').Data);
    plot(sysvector('estimator_status.covar_8').Time,sysvector('estimator_status.covar_8').Data);
    plot(sysvector('estimator_status.covar_9').Time,sysvector('estimator_status.covar_9').Data);
    hold off;
    title('Position covariances');
    state(2) = subplot(3,3,2);
    hold on;
    plot(sysvector('estimator_status.covar_0').Time,sysvector('estimator_status.covar_0').Data);
    plot(sysvector('estimator_status.covar_1').Time,sysvector('estimator_status.covar_1').Data);
    plot(sysvector('estimator_status.covar_2').Time,sysvector('estimator_status.covar_2').Data);
    plot(sysvector('estimator_status.covar_3').Time,sysvector('estimator_status.covar_3').Data);
    hold off;
    title('Angular covariances');
    state(3) = subplot(3,3,3);
    hold on;
    plot(sysvector('estimator_status.covar_4').Time,sysvector('estimator_status.covar_4').Data);
    plot(sysvector('estimator_status.covar_5').Time,sysvector('estimator_status.covar_5').Data);
    plot(sysvector('estimator_status.covar_6').Time,sysvector('estimator_status.covar_6').Data);
    hold off;
    title('Velocity covariances');
    state(4) = subplot(3,3,4);
    hold on;
    plot(sysvector('estimator_status.covar_10').Time,sysvector('estimator_status.covar_10').Data);
    plot(sysvector('estimator_status.covar_11').Time,sysvector('estimator_status.covar_11').Data);
    plot(sysvector('estimator_status.covar_12').Time,sysvector('estimator_status.covar_12').Data);
    hold off;
    title('Gyroscope bias covariances');
    state(5) = subplot(3,3,5);
    hold on;
    plot(sysvector('estimator_status.covar_13').Time,sysvector('estimator_status.covar_13').Data);
    plot(sysvector('estimator_status.covar_14').Time,sysvector('estimator_status.covar_14').Data);
    plot(sysvector('estimator_status.covar_15').Time,sysvector('estimator_status.covar_15').Data);
    hold off;
    title('Accelerometer bias covariances');
    state(6) = subplot(3,3,6);
    hold on;
    plot(sysvector('estimator_status.covar_16').Time,sysvector('estimator_status.covar_16').Data);
    plot(sysvector('estimator_status.covar_17').Time,sysvector('estimator_status.covar_17').Data);
    plot(sysvector('estimator_status.covar_18').Time,sysvector('estimator_status.covar_18').Data);
    hold off;
    title('Earth magnetic field covariances');
    state(7) = subplot(3,3,7);
    hold on;
    plot(sysvector('estimator_status.covar_22').Time,sysvector('estimator_status.covar_22').Data);
    plot(sysvector('estimator_status.covar_23').Time,sysvector('estimator_status.covar_23').Data);
    hold off;
    title('Wind covariances');
    state(8) = subplot(3,3,9);
    hold on;
    plot(sysvector('estimator_status.covar_19').Time,sysvector('estimator_status.covar_19').Data);
    plot(sysvector('estimator_status.covar_20').Time,sysvector('estimator_status.covar_20').Data);
    plot(sysvector('estimator_status.covar_21').Time,sysvector('estimator_status.covar_21').Data);
    hold off;
    title('Magnetometer bias covariances');

    linkaxes([state(1) state(2) state(3) state(4) state(5) state(6) state(7) state(8)],'x');
    set(state(:),'XGrid','on','YGrid','on','ZGrid','on');
end

% plot the relative timestamps
if topics.ekf2_timestamps.logged
    figure('Name', 'Estimator Relative Timestamps');
    timestamp_rel(1) = subplot(3,2,1);
    plot(sysvector('ekf2_timestamps.airspeed_timestamp_rel').Time,...
        sysvector('ekf2_timestamps.airspeed_timestamp_rel').Data*0.1);
    title('EKF2 Airspeed Relative Timestamp [ms]');
    
    timestamp_rel(2) = subplot(3,2,2);
    plot(sysvector('ekf2_timestamps.distance_sensor_timestamp_rel').Time,...
        sysvector('ekf2_timestamps.distance_sensor_timestamp_rel').Data*0.1);
    title('EKF2 Distance Sensor Relative Timestamp [ms]');
    
    timestamp_rel(3) = subplot(3,2,3);
    plot(sysvector('ekf2_timestamps.gps_timestamp_rel').Time,...
        sysvector('ekf2_timestamps.gps_timestamp_rel').Data*0.1);
    title('EKF2 GPS Relative Timestamp [ms]');
    
    timestamp_rel(4) = subplot(3,2,4);
    plot(sysvector('ekf2_timestamps.optical_flow_timestamp_rel').Time,...
        sysvector('ekf2_timestamps.optical_flow_timestamp_rel').Data*0.1);
    title('EKF2 Optical Flow Relative Timestamp [ms]');
    
    timestamp_rel(5) = subplot(3,2,5);
    plot(sysvector('ekf2_timestamps.vision_attitude_timestamp_rel').Time,...
        sysvector('ekf2_timestamps.vision_attitude_timestamp_rel').Data*0.1);
    title('EKF2 Vision Attitude Relative Timestamp [ms]');
    
    timestamp_rel(6) = subplot(3,2,6);
    plot(sysvector('ekf2_timestamps.vision_position_timestamp_rel').Time,...
        sysvector('ekf2_timestamps.vision_position_timestamp_rel').Data*0.1);
    title('EKF2 Vision Position Relative Timestamp [ms]');

    linkaxes([timestamp_rel(1) timestamp_rel(2) timestamp_rel(3) ...
        timestamp_rel(4) timestamp_rel(5) timestamp_rel(6)],'x');
    set(timestamp_rel(:), 'XGrid','on','YGrid','on','ZGrid','on');
end