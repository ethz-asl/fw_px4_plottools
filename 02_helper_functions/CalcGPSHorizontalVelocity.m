function [GPSVel] = CalcGPSHorizontalVelocity(sysvector, topics) 
%
%   Syntax:
%   function [GPSVel] = CalcDPSVelocity(sysvector, topics) 
%
%   Author: Philipp Oetershagen, Florian Achermann
    
    if topics.vehicle_gps_position.logged
        GPSVelData        = sqrt((sysvector('vehicle_gps_position.vel_n').Data).^2+(sysvector('vehicle_gps_position.vel_e').Data).^2);
        GPSVel = timeseries(GPSVelData, ...
            sysvector('vehicle_gps_position.vel_n').Time, ...
            'Name', 'vehicle_gps_position.velocity_horizontal');
        GPSVel.DataInfo.Interpolation = tsdata.interpolation('zoh');
    else
        GPSVel = timeseries();
    end