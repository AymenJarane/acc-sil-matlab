%% acc_params.m - parametres du plant et du controleur ACC (workspace de base)
% Script a appeler avant toute simulation : acc_params;
% Les valeurs du plant sont identiques a celles de build_acc_plant.m.

% --- Plant (vehicule ego) -------------------------------------------------
m      = 1500;    % masse [kg]
rho    = 1.225;   % densite de l'air [kg/m^3]
Cd     = 0.30;    % coefficient de trainee
Af     = 2.2;     % surface frontale [m^2]
Crr    = 0.012;   % coefficient de roulement
g      = 9.81;    % pesanteur [m/s^2]
tau_pt = 0.30;    % constante de temps traction/freinage [s]
Ts     = 0.01;    % pas fixe = periode du controleur [s]

% --- Controleur ACC -------------------------------------------------------
v_set    = 30;    % consigne de vitesse [m/s] (108 km/h)
a_max    = 3;     % acceleration max demandee [m/s^2]
a_min    = -3;    % deceleration max demandee [m/s^2]
Kp_v     = 0.5;   % PI vitesse : gain proportionnel [1/s]
Ki_v     = 0.1;   % PI vitesse : gain integral [1/s^2]
th       = 1.8;   % ecart temporel vise (time gap) [s]
d0       = 5;     % distance d'arret [m]
d_detect = 150;   % portee de detection de la cible [m]
Kd_gap   = 0.25;  % suivi : gain sur l'ecart de distance [1/s^2]
Kv_rel   = 0.8;   % suivi : gain sur la vitesse relative [1/s]