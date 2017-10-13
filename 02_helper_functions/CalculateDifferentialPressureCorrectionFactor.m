function eps = CalculateDifferentialPressureCorrectionFactor(dp, temperature, D, L, dpSensor, massflow)
% TODO add the ambient barometric pressure instead of 1.01325 in R_air

% set negative dp to 0
dp(dp < 0.0) = 0.0;

% compute correction factor
d_pow4 = D^4;

% viscosity of the air
N_air = (18.205 + 0.0484 * (temperature - 20.0)) * 1e-6;

% density of the air
R_air = (1.1885 * 1.01325 * 293.15)./(273.15 + temperature);

denominator = pi * d_pow4 * R_air.*dp;
idx = find(abs(denominator)>1e-32);
eps = zeros(size(denominator));
eps(idx) =  -64.0 * L * massflow * N_air(idx) .* (sqrt(1.0 + 8.0 * dp(idx) / dpSensor)-1.0)./denominator(idx);

% limit eps
eps(eps>=1.0) = 0.0;
eps(eps<=-1.0) = 0.0;