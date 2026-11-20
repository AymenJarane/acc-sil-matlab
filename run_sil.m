%% run_sil.m - Etape 4 : generation du code C et validation SIL (back-to-back MIL vs SIL)
% 1. Convertit le sous-systeme ACC_Controller en modele reference 'acc_ctrl_ref' (une seule fois)
% 2. Configure ce modele pour Embedded Coder (ert.tlc, langage C, pas fixe Ts)
% 3. Genere le code C -> generated_code/   (mode "code seul" : sans compilation)
% 4. SIL : pour chaque scenario, simule acc_sil avec le controleur en mode Normal (MIL),
%    puis en mode SIL (code C compile), et compare les signaux.
% Pre-requis : etapes 1 a 3 faites. La partie 4 exige un compilateur C (mex -setup C).
% ATTENTION : ce script modifie acc_sil.slx (le sous-systeme devient un bloc Model).
%             Fais un commit Git avant de le lancer.

mdl = 'acc_sil';
ref = 'acc_ctrl_ref';
tol = 1e-6;                  % tolerance back-to-back (unite du signal compare)
acc_params;
acc_scenarios(1);
load_system(mdl);
if ~exist('results', 'dir'),        mkdir('results');        end
if ~exist('generated_code', 'dir'), mkdir('generated_code'); end

%% 1. Controleur -> modele reference
blk = find_system(mdl, 'SearchDepth', 1, 'BlockType', 'ModelReference');
if isempty(blk)
    Simulink.SubSystem.convertToModelReference([mdl '/ACC_Controller'], ref, ...
        'ReplaceSubsystem', true, 'BuildTarget', 'Sim', 'AutoFix', true, 'Force', true);
    blk = find_system(mdl, 'SearchDepth', 1, 'BlockType', 'ModelReference');
end
blk = blk{1};
load_system(ref);

%% 2. Configuration Embedded Coder du modele reference
set_param(ref, 'SystemTargetFile', 'ert.tlc', 'TargetLang', 'C', ...
    'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', 'FixedStep', 'Ts', ...
    'GenerateReport', 'on');
% Entrees racine a la periode du controleur : pas de rythme continu dans le code C
ips = find_system(ref, 'SearchDepth', 1, 'BlockType', 'Inport');
for i = 1:numel(ips)
    set_param(ips{i}, 'SampleTime', 'Ts');
end
save_system(ref);
save_system(mdl);

%% 3. Generation du code C (sans compilation)
set_param(ref, 'GenCodeOnly', 'on');
slbuild(ref);
set_param(ref, 'GenCodeOnly', 'off');
save_system(ref);
save_system(mdl);

bd    = RTW.getBuildDir(ref);
src   = bd.BuildDirectory;
files = [dir(fullfile(src, '*.c')); dir(fullfile(src, '*.h'))];
fprintf('\nCode C genere (copie dans generated_code/) :\n');
for i = 1:numel(files)
    if strcmp(files(i).name, 'ert_main.c'), continue; end   % exemple de main, non utile
    copyfile(fullfile(src, files(i).name), 'generated_code');
    fprintf('  %-22s %5d lignes\n', files(i).name, ...
        numel(readlines(fullfile(src, files(i).name))));
end

%% 4. SIL : comparaison back-to-back MIL vs SIL
sc   = acc_scenarios();
n    = numel(sc);
sigs = {'a_cmd', 'v_ego', 'd'};
R    = nan(n, numel(sigs) + 1);
try
    for k = 1:n
        acc_scenarios(k);
        set_param(blk, 'SimulationMode', 'Normal');
        o_mil = sim(mdl);
        set_param(blk, 'SimulationMode', 'Software-in-the-loop (SIL)');
        o_sil = sim(mdl);
        for s = 1:numel(sigs)
            R(k, s) = max(abs(o_mil.(sigs{s}).Data - o_sil.(sigs{s}).Data));
        end
        R(k, end) = isequal(o_mil.acc_mode.Data, o_sil.acc_mode.Data);
        if k == 1, o1m = o_mil; o1s = o_sil; end
    end
catch err
    set_param(blk, 'SimulationMode', 'Normal');
    fprintf(2, '\nSIL impossible. Rapport complet (copie-le en entier) :\n%s\n', getReport(err));
    fprintf(2, 'Le code C est deja genere dans generated_code/.\n');
    return
end
set_param(blk, 'SimulationMode', 'Normal');

%% 5. Resultats
ok = all(R(:, 1:numel(sigs)) < tol, 2) & R(:, end) == 1;
T  = table(string({sc.name})', R(:,1), R(:,2), R(:,3), R(:,4) == 1, ok, ...
    'VariableNames', {'Scenario','max_diff_a_cmd','max_diff_v_ego','max_diff_d','mode_identique','OK'});
disp(T);
writetable(T, fullfile('results', 'sil_backtoback.csv'));
verdict = 'ECHEC';
if all(ok), verdict = 'REUSSI'; end
fprintf('Tolerance : %g | Verdict back-to-back MIL/SIL : %s\n', tol, verdict);

t = o1m.a_cmd.Time;
f = figure('Name', 'Back-to-back MIL vs SIL - scenario 1');
tiledlayout(2, 1);
nexttile; plot(t, o1m.a_cmd.Data, t, o1s.a_cmd.Data, '--'); grid on
ylabel('a_{cmd} [m/s^2]'); legend('MIL', 'SIL');
title('Scenario 1 : sortie du controleur, modele vs code C');
nexttile; plot(t, o1m.a_cmd.Data - o1s.a_cmd.Data); grid on
ylabel('MIL - SIL [m/s^2]'); xlabel('t [s]');
exportgraphics(f, fullfile('results', 'sil_backtoback_scenario1.png'), 'Resolution', 150);

save_system(mdl);
acc_scenarios(1);