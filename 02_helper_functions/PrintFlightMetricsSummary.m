function PrintFlightMetricsSummary(sysvector)
%PRINTFLIGHTMETRICSSUMMARY Summary of this function goes here
%   Detailed explanation goes here

iLaunchAir=find(sysvector('airspeed_0.true_airspeed_m_s').Data > 8.0,1,'first');
iLandAir=find(sysvector('airspeed_0.true_airspeed_m_s').Data > 8.0,1,'last');

if(isempty(iLaunchAir) || isempty(iLandAir))
    display('WARNING: No launch- or land-time found.');
else
    tLaunch = sysvector('airspeed_0.true_airspeed_m_s').Time(iLaunchAir);
    tLand = sysvector('airspeed_0.true_airspeed_m_s').Time(iLandAir);
    iLaunchGnd = find(sysvector('vehicle_global_position_0.vel_n').Time > tLaunch,1,'first');
    iLandGnd = find(sysvector('vehicle_global_position_0.vel_n').Time > tLand,1,'first');

    FlightTime = tLand-tLaunch;
    vAirMean = mean(sysvector('airspeed_0.true_airspeed_m_s').Data(iLaunchAir:iLandAir));
    vGnd = sqrt(sysvector('vehicle_global_position_0.vel_n').Data.^2+sysvector('vehicle_global_position_0.vel_e').Data.^2);
    vGndMean = mean(vGnd(iLaunchGnd:iLandGnd));
    FlightDistanceAir = vAirMean * FlightTime;
    FlightDistanceGnd = vGndMean * FlightTime;
    str = sprintf('tLaunch: %.2fs tLand:%.2fs tFlight: %.2fs (%s dd:hh:mm:ss) dist_air: %.2fkm dist_gnd: %.2fkm\n', tLaunch, tLand, FlightTime, datestr(FlightTime/86400.0, 'DD:HH:MM:SS'),FlightDistanceAir/1000.0,FlightDistanceGnd/1000.0);
    display(str);
end

end

