function [f, out] = Dp2Sp(x,dp_data,baro_data,temp_data,yaw_data,gspn_data,gspe_data)

D = x(1);
L = x(2);
wn = x(3);
we = x(4);

% DP reading of the sensor (101 for SDP3X, 62 for SDP600)
dpSensor = 59.3319; % [Pa]

% Massflow (4.79e-7 for SDP3X, 6.17e-7 for SDP600)
massflow = 4.79e-7; % [kg/s]

% set negative dp to 0
dp_data = abs(dp_data);

% compute correction factor
d_pow4 = D^4;

% viscosity of the air
N_air = (18.205 + 0.0484 * (temp_data - 20.0)) * 1e-6;

% density of the air
R_air = (baro_data)./(287.1 .* (273.15 + temp_data));

denominator = pi * d_pow4 * R_air.*dp_data;
idx = find(abs(denominator)>1e-32);

eps = zeros(size(denominator));
eps(idx) =  -64.0 * L * massflow * N_air(idx) .* (sqrt(1.0 + 8.0 * dp_data(idx) / dpSensor)-1.0)./denominator(idx);

% limit eps
eps(eps>=1.0) = 0.0;
eps(eps<=-1.0) = 0.0;

dp1 = dp_data./(1 + eps);

% compute indicated airspeed
airspeed_indicated = sqrt(2.0*abs(dp1) / 1.225);
airspeed_indicated(dp1 < 0.0) = 0.0;

% compute the true airspeed from the indicated airspeed
air_density = (baro_data)./(287.1 .* (273.15 + temp_data));
airspeed_true = airspeed_indicated .* sqrt(1.225 ./ air_density);

% true airsp vector
va_n = airspeed_true.*cos(yaw_data);
va_e = airspeed_true.*sin(yaw_data);

% gsp vector
gsp_n = va_n + wn;
gsp_e = va_e + we;

% % ground speed
% ground_speed = sqrt(gsp_n.^2 + gsp_e.^2);
% ground_speed_data = sqrt(gspn_data.^2 + gspe_data.^2);

% objective
f = [gspn_data - gsp_n; gspe_data - gsp_e];
% f = ground_speed_data - ground_speed;

% outputs
out = [va_n, va_e, gsp_n, gsp_e];
