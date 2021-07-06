function PrintFlightMetricsSummary(sysvector, topics)
%PRINTFLIGHTMETRICSSUMMARY Summary of this function goes here
%   Detailed explanation goes here

if (topics.airspeed.logged)
    iLaunchAir=find(sysvector.airspeed_0.true_airspeed_m_s.Data > 8.0,1,'first');
    iLandAir=find(sysvector.airspeed_0.true_airspeed_m_s.Data > 8.0,1,'last');

    if(isempty(iLaunchAir) || isempty(iLandAir))
        disp('WARNING: No launch- or land-time found.');
    else
        tLaunch = sysvector.airspeed_0.true_airspeed_m_s.Time(iLaunchAir);
        tLand = sysvector.airspeed_0.true_airspeed_m_s.Time(iLandAir);

        FlightTime = tLand-tLaunch;
        vAirMean = mean(sysvector.airspeed_0.true_airspeed_m_s.Data(iLaunchAir:iLandAir));
        if (topics.vehicle_local_position.logged)
            iLaunchGnd = find(sysvector.vehicle_local_position_0.vx.Time > tLaunch,1,'first');
            iLandGnd = find(sysvector.vehicle_local_position_0.vx.Time > tLand,1,'first');
            vGnd = sqrt(sysvector.vehicle_local_position_0.vx.Data.^2+sysvector.vehicle_local_position_0.vy.Data.^2+sysvector.vehicle_local_position_0.vz.Data.^2);
            vGndMean = mean(vGnd(iLaunchGnd:iLandGnd));
        else
           vGndMean = NaN;
        end
        FlightDistanceAir = vAirMean * FlightTime;
        FlightDistanceGnd = vGndMean * FlightTime;
        str = sprintf('***********************\n*** Flight Summary: ***\n***********************\ntLaunch: %.2f s tLand:%.2f s tFlight: %.2f s (%s dd:hh:mm:ss) dist_air: %.2f km dist_gnd: %.2f km\n', tLaunch, tLand, FlightTime, datestr(FlightTime/86400.0, 'DD:HH:MM:SS'),FlightDistanceAir/1000.0,FlightDistanceGnd/1000.0);
        disp(str);
    end

else
    disp('INFO: Airspeed topic not logged.');
end

