%% build_acc_controller.m - Etape 2 : controleur ACC reel dans acc_sil
% Remplace le contenu du sous-systeme ACC_Controller (placeholder P) par :
%   3 Zero-Order Hold (echantillonnage a Ts) + un bloc MATLAB Function
%   (PI de vitesse, PD de distance, selection du mode, saturation, anti-windup)
% Interface conservee : entrees (v_ego, d, v_rel) ; sortie 1 = a_cmd ;
% sortie 2 = acc_mode (1 = regulation de vitesse, 2 = suivi de distance).
% Pre-requis : build_acc_plant.m deja lance (acc_sil.slx dans le dossier courant).

mdl = 'acc_sil';
acc_params;          % parametres plant + controleur
acc_scenarios(1);    % leader_profile, x0_lead, v0_ego, T_sim

load_system(mdl);
ac  = [mdl '/ACC_Controller'];
pos = @(x,y,w,h) [x y x+w y+h];
rt  = {'autorouting','smart'};

%% 1. Vider le placeholder (on garde les ports d'entree/sortie)
% d'abord les fils : delete_block laisse des fils orphelins attaches aux ports
lns = find_system(ac, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'line');
for i = 1:numel(lns)
    try, delete_line(lns(i)); catch, end
end
blks = find_system(ac, 'SearchDepth', 1);
for i = 1:numel(blks)
    if strcmp(blks{i}, ac), continue; end
    if ~ismember(get_param(blks{i}, 'BlockType'), {'Inport','Outport'})
        delete_block(blks{i});
    end
end

%% 2. Code du controleur (les parametres sont inscrits comme constantes)
L = {
 'function [a_cmd, acc_mode] = acc_ctrl(v_ego, d, v_rel)'
 '% ACC : regulation de vitesse (PI) + suivi de distance (PD), selection par minimum'
 'persistent integ'
 'if isempty(integ)'
 '    integ = 0;'
 'end'
 ''
 '% --- parametres (valeurs inscrites a la construction du modele) ---'
 sprintf('v_set = %g;  a_max = %g;  a_min = %g;  Ts = %g;', v_set, a_max, a_min, Ts)
 sprintf('Kp_v = %g;  Ki_v = %g;', Kp_v, Ki_v)
 sprintf('th = %g;  d0 = %g;  d_detect = %g;', th, d0, d_detect)
 sprintf('Kd_gap = %g;  Kv_rel = %g;', Kd_gap, Kv_rel)
 ''
 '% --- regulation de vitesse : PI ---'
 'e_v      = v_set - v_ego;'
 'a_cruise = Kp_v*e_v + integ;'
 ''
 '% --- suivi de distance : ecart a la distance desiree + vitesse relative ---'
 'd_des    = d0 + th*max(v_ego, 0);'
 'a_follow = Kd_gap*(d - d_des) + Kv_rel*v_rel;'
 'target   = d < d_detect;'
 ''
 '% --- selection du mode : on retient la consigne la plus prudente ---'
 'if target && (a_follow < a_cruise)'
 '    acc_mode = 2;'
 '    a_raw    = a_follow;'
 'else'
 '    acc_mode = 1;'
 '    a_raw    = a_cruise;'
 'end'
 ''
 '% --- saturation de la demande d acceleration ---'
 'a_cmd = min(max(a_raw, a_min), a_max);'
 ''
 '% --- anti-windup : integration conditionnelle, gelee en mode suivi ---'
 'sat_hi = a_raw > a_max;'
 'sat_lo = a_raw < a_min;'
 'if acc_mode == 1 && ~((sat_hi && e_v > 0) || (sat_lo && e_v < 0))'
 '    integ = min(max(integ + Ki_v*e_v*Ts, a_min), a_max);'
 'end'
};
code = strjoin(L(:)', newline);

%% 3. Nouveau contenu du sous-systeme
set_param([ac '/v_ego'], 'Position', pos(30,48,30,16));
set_param([ac '/d'],     'Position', pos(30,98,30,16));
set_param([ac '/v_rel'], 'Position', pos(30,148,30,16));
set_param([ac '/a_cmd'], 'Position', pos(450,70,30,16));
if getSimulinkBlockHandle([ac '/acc_mode']) == -1
    add_block('simulink/Sinks/Out1', [ac '/acc_mode'], 'Position', pos(450,120,30,16));
end

add_block('simulink/Discrete/Zero-Order Hold', [ac '/zoh_v'],    'Position', pos(110,40,40,30),  'SampleTime','Ts');
add_block('simulink/Discrete/Zero-Order Hold', [ac '/zoh_d'],    'Position', pos(110,90,40,30),  'SampleTime','Ts');
add_block('simulink/Discrete/Zero-Order Hold', [ac '/zoh_vrel'], 'Position', pos(110,140,40,30), 'SampleTime','Ts');
add_block('simulink/User-Defined Functions/MATLAB Function', [ac '/acc_logic'], ...
    'Position', pos(230,40,160,140));

sf = sfroot;
cf = sf.find('-isa', 'Stateflow.EMChart', 'Path', [ac '/acc_logic']);
assert(~isempty(cf), 'Bloc MATLAB Function introuvable : acc_logic');
cf.Script = code;

add_line(ac, 'v_ego/1',     'zoh_v/1',     rt{:});
add_line(ac, 'd/1',         'zoh_d/1',     rt{:});
add_line(ac, 'v_rel/1',     'zoh_vrel/1',  rt{:});
add_line(ac, 'zoh_v/1',     'acc_logic/1', rt{:});
add_line(ac, 'zoh_d/1',     'acc_logic/2', rt{:});
add_line(ac, 'zoh_vrel/1',  'acc_logic/3', rt{:});
add_line(ac, 'acc_logic/1', 'a_cmd/1',     rt{:});
add_line(ac, 'acc_logic/2', 'acc_mode/1',  rt{:});

%% 4. Logging du mode au niveau racine
if getSimulinkBlockHandle([mdl '/log_acc_mode']) == -1
    add_block('simulink/Sinks/To Workspace', [mdl '/log_acc_mode'], ...
        'Position', pos(820,330,80,30), ...
        'VariableName','acc_mode', 'SaveFormat','Timeseries');
    add_line(mdl, 'ACC_Controller/2', 'log_acc_mode/1', rt{:});
end

save_system(mdl);

%% 5. Scenario 1 : courbes
out = sim(mdl);
t = out.v_ego.Time;
figure('Name','Etape 2 - ACC, scenario 1');
tiledlayout(4,1);
nexttile; plot(t, out.v_ego.Data*3.6, t, out.v_lead.Data*3.6); grid on
ylabel('vitesse [km/h]'); legend('ego','leader');
nexttile; plot(t, out.d.Data, t, d0 + th*out.v_ego.Data, '--'); grid on
ylabel('distance [m]'); legend('d','d desiree');
nexttile; plot(t, out.a_cmd.Data); yline([a_min a_max], 'r--'); grid on
ylabel('a_{cmd} [m/s^2]');
nexttile; stairs(t, out.acc_mode.Data); grid on
ylim([0.5 2.5]); yticks([1 2]); yticklabels({'vitesse','distance'});
xlabel('t [s]'); ylabel('mode');

%% 6. Les 5 scenarios : controle rapide
sc = acc_scenarios();
fprintf('\n%-36s %10s %10s %10s\n', 'Scenario', 'd_min [m]', 'a_min', 'v_fin [m/s]');
for k = 1:numel(sc)
    acc_scenarios(k);
    o = sim(mdl);
    fprintf('%-36s %10.1f %10.2f %10.1f\n', sc(k).name, ...
        min(o.d.Data), min(o.a_cmd.Data), o.v_ego.Data(end));
end
acc_scenarios(1);