function [GPSVel] = CalcGPSHorizontalVelocity(sysvector, topics) 
%
%   Syntax:
%   function [GPSVel] = CalcDPSVelocity(sysvector, topics) 
%
%   Author: Philipp Oetershagen, Florian Achermann
    
    if topics.vehicle_gps_position.logged
        GPSVelData        = sqrt((sysvector('vehicle_gps_position_0.vel_n_m_s').Data).^2+(sysvector('vehicle_gps_position_0.vel_e_m_s').Data).^2);
        GPSVel = timeseries(GPSVelData, ...
            sysvector('vehicle_gps_position_0.vel_n_m_s').Time, ...
            'Name', 'vehicle_gps_position.velocity_horizontal');
        GPSVel.DataInfo.Interpolation = tsdata.interpolation('zoh');
    else
        GPSVel = timeseries();
    end