function VehicleStatusFlagsPlots(sysvector, topics)
% Display the cpu load data

if ~topics.vehicle_status_flags.logged
   return 
end

fig1 = figure();
fig1.Name = 'Condition Vehicle Status Flags';

condition_flags(1) = subplot(6,2,1);
plot(sysvector.vehicle_status_flags_0.condition_calibration_enabled);
ylabel(' ')
title('Calibration Enabled')

condition_flags(2) = subplot(6,2,2);
plot(sysvector.vehicle_status_flags_0.condition_system_sensors_initialized);
ylabel(' ')
title('System Sensor Initialzed')

condition_flags(3) = subplot(6,2,3);
plot(sysvector.vehicle_status_flags_0.condition_system_hotplug_timeout);
ylabel(' ')
title('Hotplug Timeout')

condition_flags(4) = subplot(6,2,4);
plot(sysvector.vehicle_status_flags_0.condition_system_returned_to_home);
ylabel(' ')
title('Return to Home')

condition_flags(5) = subplot(6,2,5);
plot(sysvector.vehicle_status_flags_0.condition_auto_mission_available);
ylabel(' ')
title('Auto Mission Available')

condition_flags(6) = subplot(6,2,6);
plot(sysvector.vehicle_status_flags_0.condition_global_position_valid);
ylabel(' ')
title('Global Position Valid')

condition_flags(7) = subplot(6,2,7);
plot(sysvector.vehicle_status_flags_0.condition_home_position_valid);
ylabel(' ')
title('Home Position Valid')

condition_flags(8) = subplot(6,2,8);
plot(sysvector.vehicle_status_flags_0.condition_local_position_valid);
ylabel(' ')
title('Local Position Valid')

condition_flags(9) = subplot(6,2,9);
plot(sysvector.vehicle_status_flags_0.condition_local_velocity_valid);
ylabel(' ')
title('Local Velicity Valid')

condition_flags(10) = subplot(6,2,10);
plot(sysvector.vehicle_status_flags_0.condition_local_altitude_valid);
ylabel(' ')
title('Local Altitude Valid')

condition_flags(11) = subplot(6,2,11);
plot(sysvector.vehicle_status_flags_0.condition_power_input_valid);
ylabel(' ')
title('Power Input Valid')

if (isfield(sysvector, 'vehicle_land_detected_0'))
    condition_flags(12) = subplot(6,2,12);
    plot(sysvector.vehicle_land_detected_0.landed,'LineWidth',2);
    ylabel(' ')
    title('Vehicle landed state')
end

linkaxes(condition_flags(:),'x');
set(condition_flags(:), 'XGrid','on','YGrid','on','ZGrid','on');
    
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
plot(sysvector.vehicle_status_flags_0.circuit_breaker_engaged_enginefailure_check);
ylabel(' ')
title('Engine Failure Check')

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

fig3 = figure();
fig3.Name = 'Other Vehicle Status Flags';

other_flags(1) = subplot(3,2,1);
plot(sysvector.vehicle_status_flags_0.offboard_control_signal_found_once);
ylabel(' ')
title('Offboard Control Signal Found Once')

other_flags(end+1) = subplot(3,2,2);
plot(sysvector.vehicle_status_flags_0.offboard_control_signal_lost);
ylabel(' ')
title('Offboard Control Signal Lost')

other_flags(end+1) = subplot(3,2,3);
plot(sysvector.vehicle_status_flags_0.rc_signal_found_once);
ylabel(' ')
title('RC Signal Found Once')

other_flags(end+1) = subplot(3,2,4);
plot(sysvector.vehicle_status_flags_0.rc_input_blocked);
ylabel(' ')
title('RC Input Blocked')

other_flags(end+1) = subplot(3,2,5);
plot(sysvector.vehicle_status_flags_0.vtol_transition_failure);
ylabel(' ')
title('VTOL Transition Failure')

other_flags(end+1) = subplot(3,2,6);
plot(sysvector.vehicle_status_flags_0.usb_connected);
ylabel(' ')
title('USB Connected')

linkaxes(other_flags(:),'x');
set(other_flags(:), 'XGrid','on','YGrid','on','ZGrid','on');
dcm_obj = datacursormode(fig1);
set(dcm_obj,'UpdateFcn',@HighPrecisionTooltipCallback);

end

