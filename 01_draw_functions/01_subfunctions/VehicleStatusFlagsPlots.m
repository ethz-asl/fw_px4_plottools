function VehicleStatusFlagsPlots(sysvector, topics)
% Display the cpu load data

if ~topics.vehicle_status_flags.logged
   return 
end

fig1 = figure();
fig1.Name = 'Condition Vehicle Status Flags';

condition_flags(1) = subplot(6,2,1);
if isfield(sysvector.vehicle_status_flags_0, 'condition_calibration_enabled')
    plot(sysvector.vehicle_status_flags_0.condition_calibration_enabled);
else
    plot(sysvector.vehicle_status_flags_0.calibration_enabled);
end
ylabel(' ')
title('Calibration Enabled')

condition_flags(2) = subplot(6,2,2);
if isfield(sysvector.vehicle_status_flags_0, 'condition_system_sensors_initialized')
    plot(sysvector.vehicle_status_flags_0.condition_system_sensors_initialized);
    title('System Sensor Initialized')
else
    plot(sysvector.vehicle_status_flags_0.pre_flight_checks_pass);
    title('Pre Flight Checks Passed')
end
ylabel(' ')

condition_flags(3) = subplot(6,2,3);
if isfield(sysvector.vehicle_status_flags_0, 'condition_system_hotplug_timeout')
    plot(sysvector.vehicle_status_flags_0.condition_system_hotplug_timeout);
    title('Hotplug Timeout')
else
    plot(sysvector.vehicle_status_flags_0.gps_position_valid);
    title('GPS Position Valid')
end
ylabel(' ')

condition_flags(4) = subplot(6,2,4);
if isfield(sysvector.vehicle_status_flags_0, 'condition_system_returned_to_home')
    plot(sysvector.vehicle_status_flags_0.condition_system_returned_to_home);
    title('Return to Home')
else
    plot(sysvector.vehicle_status_flags_0.angular_velocity_valid);
    title('Angular Velocity Valid')
end
ylabel(' ')

condition_flags(5) = subplot(6,2,5);
if isfield(sysvector.vehicle_status_flags_0, 'condition_auto_mission_available')
    plot(sysvector.vehicle_status_flags_0.condition_auto_mission_available);
else
    plot(sysvector.vehicle_status_flags_0.auto_mission_available);
end
ylabel(' ')
title('Auto Mission Available')

condition_flags(6) = subplot(6,2,6);
if isfield(sysvector.vehicle_status_flags_0, 'condition_global_position_valid')
    plot(sysvector.vehicle_status_flags_0.condition_global_position_valid);
else
    plot(sysvector.vehicle_status_flags_0.global_position_valid);
end
ylabel(' ')
title('Global Position Valid')

condition_flags(7) = subplot(6,2,7);
if isfield(sysvector.vehicle_status_flags_0, 'condition_home_position_valid')
    plot(sysvector.vehicle_status_flags_0.condition_home_position_valid);
else
    plot(sysvector.vehicle_status_flags_0.home_position_valid);
end
ylabel(' ')
title('Home Position Valid')

condition_flags(8) = subplot(6,2,8);
if isfield(sysvector.vehicle_status_flags_0, 'condition_local_position_valid')
    plot(sysvector.vehicle_status_flags_0.condition_local_position_valid);
else
    plot(sysvector.vehicle_status_flags_0.local_position_valid);
end
ylabel(' ')
title('Local Position Valid')

condition_flags(9) = subplot(6,2,9);
if isfield(sysvector.vehicle_status_flags_0, 'condition_local_velocity_valid')
    plot(sysvector.vehicle_status_flags_0.condition_local_velocity_valid);
else
    plot(sysvector.vehicle_status_flags_0.local_velocity_valid);
end
ylabel(' ')
title('Local Velicity Valid')

condition_flags(10) = subplot(6,2,10);
if isfield(sysvector.vehicle_status_flags_0, 'condition_local_altitude_valid')
    plot(sysvector.vehicle_status_flags_0.condition_local_altitude_valid);
else
    plot(sysvector.vehicle_status_flags_0.local_altitude_valid);
end
ylabel(' ')
title('Local Altitude Valid')

condition_flags(11) = subplot(6,2,11);
if isfield(sysvector.vehicle_status_flags_0, 'condition_power_input_valid')
    plot(sysvector.vehicle_status_flags_0.condition_power_input_valid);
    title('Power Input Valid')
else
    plot(sysvector.vehicle_status_flags_0.local_position_valid_relaxed);
    title('Local Position Valid Relaxed')
end
ylabel(' ')

if (isfield(sysvector, 'vehicle_land_detected_0'))
    condition_flags(12) = subplot(6,2,12);
    plot(sysvector.vehicle_land_detected_0.landed,'LineWidth',2);
    ylabel(' ')
    title('Vehicle landed state')
elseif isfield(sysvector.vehicle_status_flags_0, 'dead_reckoning')
    condition_flags(12) = subplot(6,2,12);
    plot(sysvector.vehicle_status_flags_0.dead_reckoning);
    ylabel(' ')
    title('Dead Reckoning')
end

linkaxes(condition_flags(:),'x');
set(condition_flags(:), 'XGrid','on','YGrid','on','ZGrid','on');

if isfield(sysvector.vehicle_status_flags_0, 'circuit_breaker_engaged_power_check')
    fig2 = figure();
    fig2.Name = 'Circuit Breaker Vehicle Status Flags';

    cbk_flags(1) = subplot(3,2,1);
    plot(sysvector.vehicle_status_flags_0.circuit_breaker_engaged_power_check);
    ylabel(' ')
    title('Power Check')

    cbk_flags(end+1) = subplot(3,2,2);
    plot(sysvector.vehicle_status_flags_0.circuit_breaker_engaged_airspd_check);
    ylabel(' ')
    title('Airspeed Check')

    cbk_flags(end+1) = subplot(3,2,3);
    if isfield(sysvector.vehicle_status_flags_0, 'circuit_breaker_engaged_enginefailure_check')
        plot(sysvector.vehicle_status_flags_0.circuit_breaker_engaged_enginefailure_check);
        title('Engine Failure Check')
    else
        plot(sysvector.vehicle_status_flags_0.circuit_breaker_vtol_fw_arming_check);
        title('VTOL FW Arming Check')
    end
    ylabel(' ')

    cbk_flags(end+1) = subplot(3,2,4);
    plot(sysvector.vehicle_status_flags_0.circuit_breaker_flight_termination_disabled);
    ylabel(' ')
    title('Flight Termination Disabled')

    cbk_flags(end+1) = subplot(3,2,5);
    plot(sysvector.vehicle_status_flags_0.circuit_breaker_engaged_usb_check);
    ylabel(' ')
    title('USB Check')

    cbk_flags(end+1) = subplot(3,2,6);
    plot(sysvector.vehicle_status_flags_0.circuit_breaker_engaged_posfailure_check);
    ylabel(' ')
    title('Position Failure Check')

    linkaxes(cbk_flags(:),'x');
    set(cbk_flags(:), 'XGrid','on','YGrid','on','ZGrid','on');
end

fig3 = figure();
fig3.Name = 'Other Vehicle Status Flags';

other_flags(1) = subplot(3,2,1);
if isfield(sysvector.vehicle_status_flags_0, 'offboard_control_signal_found_once')
    plot(sysvector.vehicle_status_flags_0.offboard_control_signal_found_once);
    title('Offboard Control Signal Found Once')
else
    plot(sysvector.vehicle_status_flags_0.rc_calibration_in_progress);
    title('RC Calibration in Progress')
end
ylabel(' ')

other_flags(end+1) = subplot(3,2,2);
plot(sysvector.vehicle_status_flags_0.offboard_control_signal_lost);
ylabel(' ')
title('Offboard Control Signal Lost')

other_flags(end+1) = subplot(3,2,3);
plot(sysvector.vehicle_status_flags_0.rc_signal_found_once);
ylabel(' ')
title('RC Signal Found Once')

other_flags(end+1) = subplot(3,2,4);
if isfield(sysvector.vehicle_status_flags_0, 'rc_input_blocked')
    plot(sysvector.vehicle_status_flags_0.rc_input_blocked);
    title('RC Input Blocked')
else
    plot(sysvector.vehicle_status_flags_0.battery_low_remaining_time);
    title('Battery Low Remaining Time')
end
ylabel(' ')

other_flags(end+1) = subplot(3,2,5);
plot(sysvector.vehicle_status_flags_0.vtol_transition_failure);
ylabel(' ')
title('VTOL Transition Failure')

other_flags(end+1) = subplot(3,2,6);
if isfield(sysvector.vehicle_status_flags_0, 'usb_connected')
    plot(sysvector.vehicle_status_flags_0.usb_connected);
    title('USB Connected')
else
    plot(sysvector.vehicle_status_flags_0.battery_unhealthy);
    title('Battery Unhealthy')
end
ylabel(' ')

linkaxes(other_flags(:),'x');
set(other_flags(:), 'XGrid','on','YGrid','on','ZGrid','on');
dcm_obj = datacursormode(fig1);
set(dcm_obj,'UpdateFcn',@HighPrecisionTooltipCallback);

end

