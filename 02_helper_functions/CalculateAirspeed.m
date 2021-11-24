function [airspeed_true, airspeed_indicated] = CalculateAirspeed(dp_data, baro_data, temp_data, gyro_z_data, config)
% Calculate airspeed from the raw dp measurements and correct it according
% to the compensation model

% set negative dp to 0
dp_data(dp_data<0) = 0;
    
% density of the air
rho_air = (baro_data) ./ (287.1 .* (273.15 + temp_data));

% correct differential pressure for tube/pitot lossesd
if (config.pitot_type == 0)
    % drotek pitot
    % use tube_len
    
    dp_corr = dp_data * 96600.0 ./ baro_data;
    
    % flow through sensor
    flow_SDP33 = (300.805 - 300.878 ./ (0.00344205 * dp_corr.^0.68698 + 1.0)) .* 1.29 ./ rho_air;

    % for too small readings the compensation might result in a negative flow which causes numerical issues
    flow_SDP33(flow_SDP33<0.0) = 0.0;

    dp_pitot = (0.0032 * flow_SDP33 .* flow_SDP33 + 0.0123 * flow_SDP33 + 1.0) .* 1.29 ./ rho_air;

    % pressure drop through tube
    dp_tube = (flow_SDP33 * 0.674) / 450.0 * config.tube_len .* rho_air / 1.29;

    % speed at pitot-tube tip due to flow through sensor
    dv = 0.125 * flow_SDP33;

    % sum of all pressure drops
    dp_corr = dp_corr + dp_tube + dp_pitot;
    
elseif (config.pitot_type == 1)
    % custom pitot
    % use tube_dia + tube_len

    % DP reading of the sensor (101 for SDP3X, 62 for SDP600)
    dpSensor = 101; % [Pa]

    % Massflow (4.79e-7 for SDP3X, 6.17e-7 for SDP600)
    massflow = 4.79e-7; % [kg/s]

    % compute correction factor
    d_pow4 = config.tube_dia^4;

    % viscosity of the air
    vis_air = (18.205 + 0.0484 * (temp_data - 20.0)) * 1e-6;
    
    % denominator
    denominator = pi * d_pow4 * rho_air .* dp_data;
    idx = find(abs(denominator)>1e-32);

    eps = zeros(size(denominator));
    eps(idx) =  -64.0 * config.tube_len * massflow * vis_air(idx) .* (sqrt(1.0 + 8.0 * dp_data(idx) / dpSensor)-1.0)./denominator(idx);

    % limit eps
    eps(eps>=1.0) = 0.0;
    eps(eps<=-1.0) = 0.0;

    % differential pressure corrected for tube losses
    dp_corr = dp_data ./ (1 + eps);
    
    % speed at pitot-tube tip due to flow through sensor
    dv = 0;
else
    error('Unsupported pitot type')
end

% compute indicated airspeed
airspeed_indicated = (sqrt(2.0*abs(dp_corr) / 1.225) + dv);
airspeed_indicated(dp_corr < 0.0) = 0.0;

% correct airspeed with the scale factor
scale_factor = config.airspeed_scale_factor;
if isfield(config ,'sfPaspd')
    scale_factor = scale_factor + config.sfPaspd * airspeed_indicated;
end
if isfield(config ,'sfPgyrz')
    scale_factor = scale_factor + config.sfPgyrz * gyro_z_data;
end
airspeed_indicated = scale_factor .* airspeed_indicated;

% compute the true airspeed from the indicated airspeed
airspeed_true = airspeed_indicated .* sqrt(1.225 ./ rho_air);
end
