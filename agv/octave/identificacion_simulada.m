% IDENTIFICACION_SIMULADA  Prueba del metodo de identificacion contra la planta simulada, con los mismos
% escalones del guion de laboratorio (robot elevado). Compara lo identificado con el modelo fisico
% (motor CC + reductor) del que sale la planta: K = (Kt Vbat/(255 R))/(Kt Ke/R + b), u_dz, tau = Jw/(Kt Ke/R + b).
clear; clc; more off;
aqui = fileparts(mfilename('fullpath')); addpath(aqui);
dir_res = fullfile(aqui, '..', 'resultados'); if ~exist(dir_res, 'dir'), mkdir(dir_res); end
f_log = fullfile(dir_res, 'identificacion_simulada.txt'); if exist(f_log, 'file'), delete(f_log); end
diary(f_log);
A = agv_parametros();
U = [40 60 80 100 130 160 200 255 -60 -100 -160 -255];
pasos = [(0:numel(U)-1)' * 4 + 0.5, 1.5 * ones(numel(U),1), U'];
S = simular_agv(A, [], struct('elevado', 1, 'pasos_pwm', pasos, 't_extra', 1.0));
todos = ajustar_escalones(S.t, S.u, S.wmed);
n = rows(todos(1).escalones);
R = ajustar_escalones(S.t, S.u, S.wmed, 1:2:n);
fprintf('Identificacion con la planta simulada (ruedas elevadas, Vbat = %.1f V), ajuste con escalones impares\n', A.Vbat);
fprintf('Modelo fisico de referencia: K = %.4f rad/s/PWM x dispersion, u_dz = %.1f, tau = %.3f s\n\n', ...
        A.K_fisico, A.udz_elevado, A.tau_mec_elevado);
fprintf('Rueda  K ident.  K fisico   u_dz ident.  u_dz fisico   tau ident.  error verificacion [rad/s]\n');
nom = {'FL','FR','RL','RR'};
for i = 1:4
  Kf = A.K_fisico * A.Kt(i) / mean(A.Kt) * (A.Kt(i) == A.Kt(i));
  Ktn = A.Kt(i); Kfi = (Ktn * A.Vbat / (255 * A.R)) / (Ktn * A.Ke / A.R + A.b); udzi = 255 * A.tc_red * A.R / (Ktn * A.Vbat);
  E = todos(i).escalones; v = 2:2:rows(E);
  err = sqrt(mean((R(i).w_pred(E(v,1)) - E(v,2)).^2));
  fprintf('  %s   %.4f   %.4f     %6.1f       %6.1f       %.3f       %.3f\n', nom{i}, R(i).K, Kfi, R(i).udz, udzi, R(i).tau, err);
end
save(fullfile(dir_res, 'identificacion_simulada.mat'), 'R', 'S');
csvwrite(fullfile(dir_res, 'identificacion_simulada.csv'), [S.t', S.u', S.wmed']);
diary off;
