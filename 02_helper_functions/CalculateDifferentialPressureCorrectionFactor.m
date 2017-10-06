function eps = CalculateDifferentialPressureCorrectionFactor(dp, temperature, D, L)
% set negative dp to 0
dp(dp < 0.0) = 0.0;

% compute correction factor
d_pow4 = D^4;
N_air = (18.205 + 0.0484 * (temperature - 20.0)) * 1e-6;
R_air = (1.1885 * 1.01325 * 293.15)./(273.15 + temperature);
denominator = pi * d_pow4 * R_air.*dp;
idx = find(abs(denominator)>1e-32);
eps = zeros(size(denominator));
eps(idx) =  -64.0 * L * 6.17e-7 * N_air(idx) .* (sqrt(1.0 + 8.0 * dp(idx) / 62.0)-1.0)./denominator(idx);

% limit eps
eps(eps>=1.0) = 0.0;
eps(eps<=-1.0) = 0.0;