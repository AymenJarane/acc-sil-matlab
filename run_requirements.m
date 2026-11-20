%% run_requirements.m - Etape 5 : verification automatique des exigences
% Simule les 5 scenarios avec le controleur en mode modele (MIL) puis en mode code C (SIL)
% et verifie les exigences R1 a R5 ci-dessous.
% Sorties : tableaux dans la console + results/requirements_report.csv
% Pre-requis : run_sil deja execute (acc_sil contient le bloc Model acc_ctrl_ref).
% Attention : ce script s'execute dans le workspace de base, dont les variables
% (m, Ts, v_set...) alimentent le modele. Ne pas reutiliser ces noms ici.

mdl = 'acc_sil';
acc_params;
load_system(mdl);
if ~exist('results', 'dir'), mkdir('results'); end
blk = find_system(mdl, 'SearchDepth', 1, 'BlockType', 'ModelReference');
if isempty(blk), error('Bloc Model introuvable : lance run_sil d''abord.'); end
blk = blk{1};

%% Exigences (seuils)
% R1  Pas de collision                    : d > 0 sur toute la simulation
% R2  Ecart temporel                      : d / v_ego >= tg_req apres t_grace secondes
% R3  Acceleration demandee bornee        : a_min <= a_cmd <= a_max
% R4  Pas de depassement de la consigne   : v_ego <= v_set + v_over
% R5  Erreur statique en route libre      : |v_ego - v_set| < v_tol en fin de simulation
%                                           (applicable si le mode distance n'est jamais actif)
tg_req  = 1.5;     % ecart temporel minimal [s]
t_grace = 5;       % delai de retablissement accorde au debut de chaque scenario [s]
v_over  = 1.0;     % depassement maximal de la consigne [m/s]
v_tol   = 0.1;     % erreur statique maximale [m/s]
a_tol   = 1e-9;    % tolerance numerique sur les bornes d'acceleration

%% Simulation et verification
sc    = acc_scenarios();
modes = {'Normal', 'Software-in-the-loop (SIL)'};
label = {'MIL', 'SIL'};
R = struct('Mode', {}, 'Scenario', {}, 'd_min', {}, 'tg_min_apres_grace', {}, ...
           'a_min', {}, 'a_max', {}, 'v_max', {}, 'v_err_fin', {}, ...
           'R1_collision', {}, 'R2_ecart_temporel', {}, 'R3_accel_bornee', {}, ...
           'R4_depassement', {}, 'R5_erreur_statique', {});
try
    for im = 1:numel(modes)
        set_param(blk, 'SimulationMode', modes{im});
        for k = 1:numel(sc)
            acc_scenarios(k);
            o  = sim(mdl);
            tt = o.v_ego.Time;
            vv = o.v_ego.Data;
            dd = o.d.Data;
            aa = o.a_cmd.Data;
            md = o.acc_mode.Data;
            tg   = dd ./ max(vv, 0.1);
            late = tt >= t_grace;
            free = all(md == 1);
            R(end+1) = struct('Mode', string(label{im}), 'Scenario', k, ...
                'd_min', min(dd), 'tg_min_apres_grace', min(tg(late)), ...
                'a_min', min(aa), 'a_max', max(aa), 'v_max', max(vv), ...
                'v_err_fin', abs(vv(end) - v_set), ...
                'R1_collision', pf(min(dd) > 0), ...
                'R2_ecart_temporel', pf(min(tg(late)) >= tg_req), ...
                'R3_accel_bornee', pf(min(aa) >= a_min - a_tol && max(aa) <= a_max + a_tol), ...
                'R4_depassement', pf(max(vv) <= v_set + v_over), ...
                'R5_erreur_statique', pf(abs(vv(end) - v_set) < v_tol, free)); %#ok<SAGROW>
        end
    end
catch err
    set_param(blk, 'SimulationMode', 'Normal');
    rethrow(err);
end
set_param(blk, 'SimulationMode', 'Normal');
acc_scenarios(1);

%% Resultats
T    = struct2table(R);
vars = {'R1_collision', 'R2_ecart_temporel', 'R3_accel_bornee', 'R4_depassement', 'R5_erreur_statique'};
fprintf('\nScenarios :\n');
for k = 1:numel(sc), fprintf('  %d = %s\n', k, sc(k).name); end
fprintf('\n--- Verdicts (OK / ECHEC / n/a = non applicable) ---\n');
disp(T(:, [{'Mode', 'Scenario'}, vars]));
fprintf('\n--- Mesures ---\n');
disp(T(:, {'Mode', 'Scenario', 'd_min', 'tg_min_apres_grace', 'a_min', 'a_max', 'v_max', 'v_err_fin'}));
writetable(T, fullfile('results', 'requirements_report.csv'));

verdicts = T{:, vars};
if any(verdicts == "ECHEC", 'all')
    fprintf(2, 'VERDICT : AU MOINS UNE EXIGENCE EST EN ECHEC\n');
else
    fprintf('VERDICT : TOUTES LES EXIGENCES SONT VERIFIEES (MIL et SIL)\n');
end
fprintf('Seuils : ecart temporel >= %.1f s apres %g s | depassement <= %g m/s | erreur statique < %g m/s\n', ...
    tg_req, t_grace, v_over, v_tol);

%% Fonction locale
function s = pf(ok, applicable)
if nargin > 1 && ~applicable
    s = "n/a";
elseif ok
    s = "OK";
else
    s = "ECHEC";
end
end