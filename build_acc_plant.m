%% build_acc_plant.m - Etape 1 du mini-projet SIL : plant ACC (ego + leader)
% Construit le modele Simulink 'acc_sil' :
%   Leader         -> v_lead, x_lead
%   Ego_Vehicle    -> chaine de traction (1er ordre) + resistances + cinematique
%   ACC_Controller -> PLACEHOLDER (P sur la vitesse), a remplacer a l'etape 2
%   d = x_lead - x_ego ; v_rel = v_lead - v_ego ; signaux logges via To Workspace
% Lancer dans un dossier de travail dedie (acc_sil.slx y sera cree).

clear; clc;
mdl = 'acc_sil';

%% 1. Parametres (workspace de base)
m         = 1500;    % masse [kg]
rho       = 1.225;   % densite de l'air [kg/m^3]
Cd        = 0.30;    % coefficient de trainee
Af        = 2.2;     % surface frontale [m^2]
Crr       = 0.012;   % coefficient de roulement
g         = 9.81;    % pesanteur [m/s^2]
tau_pt    = 0.30;    % constante de temps traction/freinage [s]
Ts        = 0.01;    % pas fixe = periode du controleur [s]

v_set     = 30;      % consigne de vitesse [m/s] (108 km/h)
a_max     = 3;       % saturation d'acceleration du controleur [m/s^2]
Kp_cruise = 0.5;     % gain du placeholder (a remplacer par un PI, etape 2)

acc_scenarios(1);    % charge leader_profile, x0_lead, v0_ego, T_sim

%% 2. Modele
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);
set_param(mdl, 'SolverType','Fixed-step', 'Solver','ode4', 'FixedStep','Ts', ...
               'StopTime','T_sim', 'ReturnWorkspaceOutputs','on');
pos = @(x,y,w,h) [x y x+w y+h];
rt  = {'autorouting','smart'};

% --- Leader : profil de vitesse + integrateur de position -----------------
ld = [mdl '/Leader'];
add_block('built-in/SubSystem', ld, 'Position', pos(40,40,110,60));
add_block('simulink/Sources/From Workspace', [ld '/profil_v'], 'Position', pos(30,30,90,30), ...
    'VariableName','leader_profile', 'Interpolate','on', ...
    'OutputAfterFinalValue','Holding final value');
add_block('simulink/Continuous/Integrator', [ld '/int_x'], 'Position', pos(170,60,30,30), ...
    'InitialCondition','x0_lead');
add_block('simulink/Sinks/Out1', [ld '/v_lead'], 'Position', pos(260,35,30,16));
add_block('simulink/Sinks/Out1', [ld '/x_lead'], 'Position', pos(260,65,30,16));
add_line(ld, 'profil_v/1', 'v_lead/1', rt{:});
add_line(ld, 'profil_v/1', 'int_x/1',  rt{:});
add_line(ld, 'int_x/1',    'x_lead/1', rt{:});

% --- Ego : a_cmd -> traction (1er ordre) -> bilan des forces -> v, x -------
eg = [mdl '/Ego_Vehicle'];
add_block('built-in/SubSystem', eg, 'Position', pos(560,120,120,80));
add_block('simulink/Sources/In1', [eg '/a_cmd'], 'Position', pos(30,48,30,16));
add_block('simulink/Continuous/Transfer Fcn', [eg '/chaine_traction'], 'Position', pos(100,35,70,40), ...
    'Numerator','1', 'Denominator','[tau_pt 1]');
add_block('simulink/User-Defined Functions/Fcn', [eg '/resistances'], 'Position', pos(200,130,150,30), ...
    'Expr','(0.5*rho*Cd*Af*u^2 + Crr*m*g*sgn(u))/m');
add_block('simulink/Math Operations/Sum', [eg '/bilan'], 'Position', pos(240,45,25,25), ...
    'Inputs','+-');
add_block('simulink/Continuous/Integrator', [eg '/int_v'], 'Position', pos(300,45,30,30), ...
    'InitialCondition','v0_ego', 'LimitOutput','on', ...
    'UpperSaturationLimit','inf', 'LowerSaturationLimit','0');
add_block('simulink/Continuous/Integrator', [eg '/int_x'], 'Position', pos(380,45,30,30), ...
    'InitialCondition','0');
add_block('simulink/Sinks/Out1', [eg '/v_ego'], 'Position', pos(460,18,30,16));
add_block('simulink/Sinks/Out1', [eg '/x_ego'], 'Position', pos(460,52,30,16));
add_block('simulink/Sinks/Out1', [eg '/a_act'], 'Position', pos(460,86,30,16));
add_line(eg, 'a_cmd/1',           'chaine_traction/1', rt{:});
add_line(eg, 'chaine_traction/1', 'bilan/1',           rt{:});
add_line(eg, 'chaine_traction/1', 'a_act/1',           rt{:});
add_line(eg, 'resistances/1',     'bilan/2',           rt{:});
add_line(eg, 'bilan/1',           'int_v/1',           rt{:});
add_line(eg, 'int_v/1',           'resistances/1',     rt{:});
add_line(eg, 'int_v/1',           'int_x/1',           rt{:});
add_line(eg, 'int_v/1',           'v_ego/1',           rt{:});
add_line(eg, 'int_x/1',           'x_ego/1',           rt{:});

% --- Controleur ACC : PLACEHOLDER (regulation de vitesse P seulement) -----
ac = [mdl '/ACC_Controller'];
add_block('built-in/SubSystem', ac, 'Position', pos(330,120,120,80));
set_param(ac, 'TreatAsAtomicUnit', 'on');    % prepare la generation de code (etape 4)
add_block('simulink/Sources/In1', [ac '/v_ego'], 'Position', pos(30,48,30,16));
add_block('simulink/Sources/In1', [ac '/d'],     'Position', pos(30,98,30,16));
add_block('simulink/Sources/In1', [ac '/v_rel'], 'Position', pos(30,148,30,16));
add_block('simulink/Sources/Constant', [ac '/v_set_cst'], 'Position', pos(30,10,50,20), ...
    'Value','v_set');
add_block('simulink/Math Operations/Sum', [ac '/err_v'], 'Position', pos(120,30,25,25), ...
    'Inputs','+-');
add_block('simulink/Math Operations/Gain', [ac '/Kp'], 'Position', pos(180,30,40,25), ...
    'Gain','Kp_cruise');
add_block('simulink/Discontinuities/Saturation', [ac '/sat_a'], 'Position', pos(250,30,40,25), ...
    'UpperLimit','a_max', 'LowerLimit','-a_max');
add_block('simulink/Sinks/Out1', [ac '/a_cmd'], 'Position', pos(330,33,30,16));
add_block('simulink/Sinks/Terminator', [ac '/term_d'],    'Position', pos(120,98,20,16));
add_block('simulink/Sinks/Terminator', [ac '/term_vrel'], 'Position', pos(120,148,20,16));
add_line(ac, 'v_set_cst/1', 'err_v/1',     rt{:});
add_line(ac, 'v_ego/1',     'err_v/2',     rt{:});
add_line(ac, 'err_v/1',     'Kp/1',        rt{:});
add_line(ac, 'Kp/1',        'sat_a/1',     rt{:});
add_line(ac, 'sat_a/1',     'a_cmd/1',     rt{:});
add_line(ac, 'd/1',         'term_d/1',    rt{:});
add_line(ac, 'v_rel/1',     'term_vrel/1', rt{:});

% --- Niveau racine : distance, vitesse relative, cablage ------------------
add_block('simulink/Math Operations/Sum', [mdl '/calc_d'],    'Position', pos(250,60,25,25),  'Inputs','+-');
add_block('simulink/Math Operations/Sum', [mdl '/calc_vrel'], 'Position', pos(250,10,25,25),  'Inputs','+-');
add_line(mdl, 'Leader/2',      'calc_d/1',        rt{:});   % x_lead
add_line(mdl, 'Ego_Vehicle/2', 'calc_d/2',        rt{:});   % x_ego
add_line(mdl, 'Leader/1',      'calc_vrel/1',     rt{:});   % v_lead
add_line(mdl, 'Ego_Vehicle/1', 'calc_vrel/2',     rt{:});   % v_ego
add_line(mdl, 'Ego_Vehicle/1', 'ACC_Controller/1', rt{:});
add_line(mdl, 'calc_d/1',      'ACC_Controller/2', rt{:});
add_line(mdl, 'calc_vrel/1',   'ACC_Controller/3', rt{:});
add_line(mdl, 'ACC_Controller/1', 'Ego_Vehicle/1', rt{:});

% --- Logging (variables out.v_ego, out.d, ... apres sim) ------------------
logs = {'v_ego',  'Ego_Vehicle/1';
        'v_lead', 'Leader/1';
        'd',      'calc_d/1';
        'v_rel',  'calc_vrel/1';
        'a_cmd',  'ACC_Controller/1';
        'a_act',  'Ego_Vehicle/3'};
for i = 1:size(logs,1)
    b = ['log_' logs{i,1}];
    add_block('simulink/Sinks/To Workspace', [mdl '/' b], ...
        'Position', pos(820, 30+50*(i-1), 80, 30), ...
        'VariableName', logs{i,1}, 'SaveFormat','Timeseries');
    add_line(mdl, logs{i,2}, [b '/1'], rt{:});
end

save_system(mdl);
open_system(mdl);

%% 3. Simulation du scenario 1 avec le placeholder
out = sim(mdl);

t = out.v_ego.Time;
figure('Name','Etape 1 - plant + placeholder');
tiledlayout(3,1);
nexttile; plot(t, out.v_ego.Data*3.6, t, out.v_lead.Data*3.6); grid on
ylabel('vitesse [km/h]'); legend('ego','leader');
nexttile; plot(t, out.d.Data); yline(0,'r--'); grid on
ylabel('distance [m]');
nexttile; plot(t, out.a_cmd.Data, t, out.a_act.Data); grid on
ylabel('acceleration [m/s^2]'); xlabel('t [s]'); legend('a_{cmd}','a_{act}');

fprintf('d_min = %.1f m  (d < 0 => collision)\n', min(out.d.Data));