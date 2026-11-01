%% run_mil.m - Etape 3 : validation MIL (Model-in-the-Loop) des 5 scenarios
% Simule acc_sil pour chaque scenario, calcule des indicateurs, trace et enregistre :
%   results/mil_scenario_k.png   courbes de chaque scenario
%   results/mil_summary.csv      tableau d'indicateurs
%   results/mil_reference.mat    signaux de reference (comparaison back-to-back, etape 4)
% Pre-requis : build_acc_plant.m et build_acc_controller.m deja executes.

mdl = 'acc_sil';
acc_params;
load_system(mdl);
if ~exist('results', 'dir'), mkdir('results'); end

sc  = acc_scenarios();
n   = numel(sc);
res = cell(1, n);
M   = zeros(n, 8);

%% 1. Simulation et indicateurs
for k = 1:n
    acc_scenarios(k);
    o = sim(mdl);
    r = struct('name', sc(k).name, 't', o.v_ego.Time, ...
        'v_ego', o.v_ego.Data, 'v_lead', o.v_lead.Data, 'd', o.d.Data, ...
        'v_rel', o.v_rel.Data, 'a_cmd', o.a_cmd.Data, 'a_act', o.a_act.Data, ...
        'acc_mode', o.acc_mode.Data);

    tg   = r.d ./ max(r.v_ego, 0.1);          % ecart temporel [s]
    idx  = find(tg < 1.5, 1, 'last');         % dernier instant sous 1,5 s
    t_ok = 0;
    if ~isempty(idx), t_ok = r.t(idx); end

    M(k,:) = [min(r.d), min(tg), t_ok, min(r.a_cmd), max(r.a_cmd), ...
              r.v_ego(end), 100*mean(r.acc_mode == 2), sum(diff(r.acc_mode) ~= 0)];
    res{k} = r;
    plot_scenario(r, k, d0, th, a_min, a_max);
end
mil = [res{:}];

%% 2. Tableau de synthese
T = table(string({sc.name})', M(:,1), M(:,2), M(:,3), M(:,4), M(:,5), M(:,6), M(:,7), M(:,8), ...
    'VariableNames', {'Scenario','d_min','tg_min','t_tg_ok','a_min','a_max','v_fin','pct_suivi','n_commut'});
disp(T);
fprintf('d_min [m] | tg_min : ecart temporel minimal [s] | t_tg_ok : dernier instant avec ecart < 1,5 s [s]\n');
fprintf('a_min, a_max [m/s^2] | v_fin [m/s] | pct_suivi : part du temps en mode distance [%%] | n_commut : changements de mode\n');
writetable(T, fullfile('results', 'mil_summary.csv'));

%% 3. Reference MIL pour l'etape 4
info = struct('date', datestr(now), 'release', version('-release'), 'Ts', Ts);
save(fullfile('results', 'mil_reference.mat'), 'mil', 'info');
acc_scenarios(1);

%% Fonctions locales
function plot_scenario(r, k, d0, th, a_min, a_max)
f  = figure('Name', sprintf('MIL - scenario %d', k), 'Position', [100 100 780 760]);
tl = tiledlayout(4, 1, 'TileSpacing', 'compact');
title(tl, sprintf('Scenario %d : %s', k, r.name));

nexttile; plot(r.t, r.v_ego*3.6, r.t, r.v_lead*3.6); grid on
ylabel('vitesse [km/h]'); legend('ego', 'leader', 'Location', 'best');

nexttile; plot(r.t, r.d, r.t, d0 + th*r.v_ego, '--'); grid on
ylabel('distance [m]'); legend('d', 'd desiree', 'Location', 'best');

nexttile; plot(r.t, r.a_cmd); yline([a_min a_max], 'r--'); grid on
ylabel('a_{cmd} [m/s^2]');

nexttile; stairs(r.t, r.acc_mode); grid on
ylim([0.5 2.5]); yticks([1 2]); yticklabels({'vitesse', 'distance'});
ylabel('mode'); xlabel('t [s]');

exportgraphics(f, fullfile('results', sprintf('mil_scenario_%d.png', k)), 'Resolution', 150);
end