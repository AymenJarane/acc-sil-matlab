function sc = acc_scenarios(k)
%ACC_SCENARIOS  Scenarios de test du mini-projet ACC (SIL).
%   sc = acc_scenarios()   -> struct array des 5 scenarios
%   acc_scenarios(k)       -> charge le scenario k dans le workspace de base
%                             (leader_profile, x0_lead, v0_ego, T_sim)
%
%   Champs : name, T [s], profile [t v_lead], x0_lead [m], v0_ego [m/s]

t_sin = (0:0.1:80)';

sc = [ ...
    mk('Leader qui freine puis re-accelere', 80, [0 20 26 50 56 80], [25 25 10 10 25 25],      60,  25), ...
    mk('Route libre',                        40, [0 40],             [33 33],                   300, 20), ...
    mk('Vehicule lent proche (type cut-in)', 40, [0 40],             [17 17],                   35,  25), ...
    mk('Leader sinusoidal',                  80, t_sin,              20 + 4*sin(2*pi*t_sin/20), 50,  20), ...
    mk('Rattrapage d''un leader lent',       60, [0 60],             [20 20],                   150, 30)];

if nargin > 0
    assignin('base', 'leader_profile', sc(k).profile);
    assignin('base', 'x0_lead',        sc(k).x0_lead);
    assignin('base', 'v0_ego',         sc(k).v0_ego);
    assignin('base', 'T_sim',          sc(k).T);
    fprintf('Scenario %d charge : %s\n', k, sc(k).name);
end
end

function s = mk(name, T, t, v, x0, v0)
s = struct('name', name, 'T', T, 'profile', [t(:) v(:)], ...
           'x0_lead', x0, 'v0_ego', v0);
end