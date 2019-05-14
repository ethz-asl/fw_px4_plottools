% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
% Sensitivity of airflow measurements to flow disruption via proximity to
% fuselage and/or wing profile
% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

clear; clc;

% atmospheric parameters
R_inf = 287.1; % J/kg/K
p_inf = 100000; % Pa
T_inf = 273.15 + 23; % K
rho_inf = p_inf / T_inf / R_inf; % kg/m^3
RT_inf = R_inf * T_inf;

% true airspeed range
v_inf_set = linspace(0,30,101);

% profile/pitot mounting geometry
radius_profile_cm = 5;
r_probe_tip_cm = 12.5;
theta_probe_tip_deg = 0;
mount_location = 1; % 0 = wing (2D cylinder assumption), 1 = nose (3D sphere assumption)

% potential flow scales (cylinder)
if (mount_location == 0)
    kv = 1 / ( ...
        (1 - radius_profile_cm^2 / r_probe_tip_cm^2) * cosd(theta_probe_tip_deg)^2 + ...
        (1 + radius_profile_cm^2 / r_probe_tip_cm^2) * sind(theta_probe_tip_deg)^2 );
elseif (mount_location == 1)
% potential flow scales (sphere)
    kv = 1 / ( ...
        (1 - radius_profile_cm^3 / r_probe_tip_cm^3) * cosd(theta_probe_tip_deg)^2 + ...
        (1 + radius_profile_cm^3 / r_probe_tip_cm^3 / 2) * sind(theta_probe_tip_deg)^2 );
else
    disp('ERROR: not a valid mounting location');
end

% local measurements
v_l = v_inf_set / kv;
dp_l = 1/2 * rho_inf * v_l.^2;
p_l = p_inf + 1/2 * rho_inf * v_inf_set.^2 * (1 - (1/kv)^2);

% recalculated velocity
v_inf_recalc = sqrt( (2*RT_inf*dp_l) ./ (p_l - dp_l*(kv^2-1)) ) * kv;
rho_approx = p_l / RT_inf;
v_inf_noscale = sqrt(2*dp_l./rho_approx);
v_inf_approx = sqrt(2*dp_l./rho_approx) * 1.0684;

%% plot
figure('color','w');

subplot(1,2,1); hold on; grid on; box on;
plot(v_inf_set, v_inf_recalc);
plot(v_inf_set, v_inf_noscale);
plot(v_inf_set, v_inf_approx);
legend('truth', 'no scale', 'linear scale');
ylabel('v_{\infty} approx. [m/s]');
xlabel('v_{\infty} truth [m/s]');

subplot(1,2,2); hold on; grid on; box on;
plot(v_inf_set, v_inf_recalc - v_inf_set);
plot(v_inf_set, v_inf_noscale - v_inf_set);
plot(v_inf_set, v_inf_approx - v_inf_set);
legend('zero', 'no scale', 'linear scale');
ylabel('v_{\infty} approx. error [m/s]');
xlabel('v_{\infty} truth [m/s]');
