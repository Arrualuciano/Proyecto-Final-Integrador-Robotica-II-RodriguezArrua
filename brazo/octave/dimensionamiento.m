% DIMENSIONAMIENTO  Sosten estatico, exigencia de los accionamientos y carga admisible (enunciado §4.1 y §4.4).
%  - Recorre una grilla de posturas dentro de los limites (q2, q3, q4, q5; q1 y q6 no afectan la gravedad)
%    y busca el maximo |g_i(q)| de cada junta con la carga maxima del JSON.
%  - Lleva el par de junta al motor: tau_m = tau / (eta N)   (Tema III, §3.15)
%  - Carga admisible: mayor masa en el TCP tal que |g_i(q, m_p)| <= eta N tau_m,max / FS en toda la grilla.
%  - FS = 1,5 cubre los pares de aceleracion (se verifican con las trayectorias) y la caida del par
%    del paso a paso con la velocidad.
clear; clc;
aqui = fileparts(mfilename('fullpath')); addpath(aqui);
dir_res = fullfile(aqui, '..', 'resultados');
f_log = fullfile(dir_res, 'dimensionamiento.txt'); if exist(f_log, 'file'), delete(f_log); end
diary(f_log);
P = cargar_parametros(); dyn = modelo_dinamico_brazo(P);
FS = 1.5; mmax = P.brazo.carga_max;
cap = dyn.eta .* dyn.N .* dyn.tau_motor_max;               % par de junta disponible [N m]

q2v = linspace(P.brazo.q_min(2), P.brazo.q_max(2), 21);
q3v = linspace(P.brazo.q_min(3), P.brazo.q_max(3), 21);
q4v = deg2rad([0 45 90]);
q5v = linspace(P.brazo.q_min(5), P.brazo.q_max(5), 11);
n = numel(q2v) * numel(q3v) * numel(q4v) * numel(q5v);
G0 = zeros(6, n); G1 = zeros(6, n); Qg = zeros(6, n); k = 0;
for q2 = q2v, for q3 = q3v, for q4 = q4v, for q5 = q5v
  k = k + 1; q = [0; q2; q3; q4; q5; 0]; Qg(:, k) = q;
  G0(:, k) = rne_brazo(q, zeros(6,1), zeros(6,1), P, dyn, struct('carga', 0, 'rotor', false));
  G1(:, k) = rne_brazo(q, zeros(6,1), zeros(6,1), P, dyn, struct('carga', 1, 'rotor', false)) - G0(:, k);
end, end, end, end
Gm = G0 + mmax * G1;                                        % g es lineal en la masa de la pieza

fprintf('=== Dimensionamiento estatico (%d posturas, pieza %.2f kg, FS = %.1f) ===\n', n, mmax, FS);
fprintf('Junta  max|g| [N m]  postura (q2,q3,q4,q5) [deg]     N   eta  tau_m req [N m]  tau_m max  uso con FS\n');
tab = zeros(6, 6);
for i = 2:6
  [gmax, kk] = max(abs(Gm(i, :)));
  tm = gmax / (dyn.eta(i) * dyn.N(i));
  uso = FS * tm / dyn.tau_motor_max(i);
  fprintf('  %d    %8.3f      (%5.0f %5.0f %5.0f %5.0f)        %3d  %.2f   %8.4f      %6.3f    %5.0f %%\n', ...
          i, gmax, rad2deg(Qg(2:5, kk)), dyn.N(i), dyn.eta(i), tm, dyn.tau_motor_max(i), 100 * uso);
  tab(i, :) = [i, gmax, dyn.N(i), tm, dyn.tau_motor_max(i), uso];
end
fprintf('  1    (eje vertical: la gravedad no genera par; lo dimensiona la aceleracion de la base)\n');

% Carga admisible por junta
fprintf('\nCarga admisible en el TCP (toda la grilla, criterio |g_i| <= eta N tau_m,max / FS):\n');
madm = inf(6,1);
for i = 2:6
  C = cap(i) / FS; a = G0(i, :); b = G1(i, :);
  lim = inf(size(a));
  pos = b > 1e-12; neg = b < -1e-12;
  lim(pos) = (C - a(pos)) ./ b(pos);
  lim(neg) = (C + a(neg)) ./ (-b(neg));
  madm(i) = max(0, min(lim));
  if any(abs(a) > C), madm(i) = 0; end
  fprintf('   junta %d: m_p <= %6.3f kg\n', i, madm(i));
end
[mlim, jlim] = min(madm);
fprintf('   => carga admisible %.3f kg, limitada por la junta %d (postura mas desfavorable de toda la grilla)\n', mlim, jlim);

% Lo mismo restringido a la tarea: posturas usadas en toma/colocacion
S = load(fullfile(dir_res, 'cinematica_tarea.mat'));
fprintf('\nEn las poses de la tarea, con la pieza de %.2f kg, uso de cada motor (con FS):\n', P.pieza.masa);
for k = 1:size(S.Q_tarea, 2)
  g = rne_brazo(S.Q_tarea(:,k), zeros(6,1), zeros(6,1), P, dyn, struct('carga', P.pieza.masa));
  uso = FS * abs(g) ./ cap;
  fprintf('   %-18s %s %%\n', S.poses{k,1}, mat2str(round(100 * uso'), 3));
end

% Velocidad del motor
wm = dyn.N .* P.brazo.qd_max;
fprintf('\nVelocidad del motor a la velocidad articular maxima: %s rpm\n', mat2str(round(wm' * 60 / (2*pi))));
csvwrite(fullfile(dir_res, 'dimensionamiento.csv'), tab);
diary off;
