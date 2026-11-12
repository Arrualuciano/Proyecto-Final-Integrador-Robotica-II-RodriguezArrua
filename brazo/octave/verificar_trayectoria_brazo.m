% VERIFICAR_TRAYECTORIA_BRAZO  Genera el ciclo de la tarea y comprueba (enunciado §4.5.2-3 y §7.3):
%  1) duraciones, limites articulares de posicion, velocidad y aceleracion;
%  2) continuidad: derivada numerica de q vs qd, saltos entre muestras, error de la recta en LIN;
%  3) distancia a las tres singularidades y holgura vertical de codo, muneca, brida y dedos;
%  4) pares articulares y de motor a lo largo del ciclo (con pieza, friccion e inercia de rotor),
%     pico y RMS, velocidad de motor; contraste dinamica_rapida vs Newton-Euler de referencia.
% Guarda resultados/ciclo_brazo.mat, resultados/ciclo_brazo.csv, figuras y el registro de texto.
clear; clc; more off;
aqui = fileparts(mfilename('fullpath')); addpath(aqui);
dir_res = fullfile(aqui, '..', 'resultados'); dir_fig = fullfile(dir_res, 'figuras');
if ~exist(dir_fig, 'dir'), mkdir(dir_fig); end
f_log = fullfile(dir_res, 'verificacion_trayectoria.txt'); if exist(f_log, 'file'), delete(f_log); end
diary(f_log);
P = cargar_parametros(); dyn = modelo_dinamico_brazo(P);
dt = 5e-3;
fprintf('=== Trayectorias del brazo (%s), dt = %g s, margen %.2f ===\n\n', datestr(now), dt, P.brazo.tarea.margen_tiempo);
tic; C = generar_ciclo_brazo(P, dt); fprintf('Generacion: %.1f s\n', toc);

%% 1) Tramos y limites
fprintf('\n1) Tramos\n');
for k = 1:numel(C.nombres)
  fprintf('   %d %-36s T = %5.2f s\n', k, C.nombres{k}, C.T_seg(k));
end
fprintf('   Duracion total del ciclo: %.2f s (requisito R-B-7: <= 12 s)\n', C.t(end));
for k = 1:numel(C.info_lin)
  I = C.info_lin{k};
  fprintf('   LIN %d: recorrido %.3f m, giro %.1f deg, alargues %d, error max de la recta %.1e m, min|det J| %.2e\n', ...
          k, I.d, rad2deg(I.theta), I.alargues, I.e_max, I.det_min);
end
fprintf('\n   Junta | rango usado [deg]   | limites [deg]   | max|qd| [deg/s] (uso) | max|qdd| [deg/s2] (uso)\n');
for i = 1:6
  fprintf('     %d   | %7.1f a %7.1f | %5.0f a %5.0f | %6.1f (%3.0f %%)       | %7.1f (%3.0f %%)\n', i, ...
          rad2deg(min(C.q(i,:))), rad2deg(max(C.q(i,:))), rad2deg(P.brazo.q_min(i)), rad2deg(P.brazo.q_max(i)), ...
          rad2deg(max(abs(C.qd(i,:)))), 100 * max(abs(C.qd(i,:))) / P.brazo.qd_max(i), ...
          rad2deg(max(abs(C.qdd(i,:)))), 100 * max(abs(C.qdd(i,:))) / P.brazo.qdd_max(i));
end
dentro = all(all(C.q >= P.brazo.q_min - 1e-9 & C.q <= P.brazo.q_max + 1e-9));
fprintf('   Posiciones dentro de limites: %s\n', mat2str(dentro));

%% 2) Continuidad
qd_num = gradient(C.q, dt); qdd_num = gradient(C.qd, dt);
fprintf('\n2) Continuidad: max|dq/dt numerica - qd| = %.2e rad/s ; max|dqd/dt - qdd| = %.2e rad/s2 ; max salto |dq| = %.3f deg\n', ...
        max(abs(qd_num(:) - C.qd(:))), max(abs(qdd_num(:) - C.qdd(:))), rad2deg(max(max(abs(diff(C.q, 1, 2))))));

%% 3) Singularidades y holguras
N = numel(C.t); rho = zeros(1,N); fac = zeros(3,N); zmin = zeros(4,N); ptcp = zeros(3,N);
M_ = P.brazo.dh_mat; a2 = M_(2,3); d4 = M_(4,2); ht = P.brazo.herramienta.tcp_z6;
for k = 1:N
  q = C.q(:,k);
  [Tt, T] = mcd_brazo(q, P);
  fac(:,k) = [a2*sin(q(2)) + d4*cos(q(2)+q(3)); cos(q(3)); sin(q(5))];
  dedos = T(1:3,4,7) + (ht + 0.015) * T(1:3,3,7);
  zmin(:,k) = [T(3,4,3) - 0.032; T(3,4,5) - 0.020; T(3,4,7) - 0.020; dedos(3)];   % radio de cada pieza
  ptcp(:,k) = Tt(1:3,4);
end
fprintf('\n3) Singularidades (minimos a lo largo del ciclo): |rho| = %.3f m, |cos q3| = %.3f, |sin q5| = %.3f\n', ...
        min(abs(fac(1,:))), min(abs(fac(2,:))), min(abs(fac(3,:))));
fprintf('   Holgura vertical minima sobre el plano de apoyo z0 = 0: codo %.3f m, muneca %.3f m, brida %.3f m, dedos %.3f m\n', ...
        min(zmin(1,:)), min(zmin(2,:)), min(zmin(3,:)), min(zmin(4,:)));

%% 4) Dinamica a lo largo del ciclo
D0 = preparar_dinamica(P, dyn, 0); Dp = preparar_dinamica(P, dyn, C.mp_pieza);
tau = zeros(6,N); tau_g = zeros(6,N);
for k = 1:N
  if C.mp(k) > 0, D = Dp; else, D = D0; end
  [Mk, hk] = dinamica_rapida(C.q(:,k), C.qd(:,k), D);
  tf = D.b .* C.qd(:,k) + D.tau_c .* tanh(C.qd(:,k) / 1e-3);
  tau(:,k) = Mk * C.qdd(:,k) + hk + tf;
  [~, hg] = dinamica_rapida(C.q(:,k), zeros(6,1), D); tau_g(:,k) = hg;
end
% contraste con la implementacion de referencia en 5 muestras
idx = round(linspace(1, N, 7)); idx = idx(2:end-1); e_ref = 0;
for k = idx
  tr = rne_brazo(C.q(:,k), C.qd(:,k), C.qdd(:,k), P, dyn, struct('carga', C.mp(k), 'friccion', true));
  e_ref = max(e_ref, max(abs(tr - tau(:,k))));
end
fprintf('\n4) Pares a lo largo del ciclo (pieza %.2f kg, friccion y rotores incluidos). Contraste con rne_brazo: %.1e N m\n', C.mp_pieza, e_ref);
tau_m = tau ./ (D0.eta .* D0.N);
rms_ = sqrt(trapz(C.t, tau.^2, 2) / C.t(end));
fprintf('   Junta | pico |tau| [N m] | RMS [N m] | pico sosten g [N m] | tau_m pico [N m] | uso pico / RMS motor | motor max [rpm]\n');
for i = 1:6
  fprintf('     %d   |   %7.3f       |  %6.3f   |     %7.3f         |     %6.4f       |   %3.0f %% / %3.0f %%      |   %4.0f\n', i, ...
          max(abs(tau(i,:))), rms_(i), max(abs(tau_g(i,:))), max(abs(tau_m(i,:))), ...
          100 * max(abs(tau_m(i,:))) / D0.tau_motor_max(i), 100 * rms_(i) / (D0.eta(i) * D0.N(i)) / D0.tau_motor_max(i), ...
          max(abs(C.qd(i,:))) * D0.N(i) * 60 / (2*pi));
end
fprintf('   Criterio: pico <= 80 %% (caida de par del paso a paso con la velocidad), RMS <= 50 %% (calentamiento).\n');

%% Guardar
save(fullfile(dir_res, 'ciclo_brazo.mat'), 'C', 'tau', 'tau_g');
csvwrite(fullfile(dir_res, 'ciclo_brazo.csv'), [C.t', C.q', C.qd', C.qdd', C.mp', C.seg', tau']);
fprintf('\nCSV: columnas t, q1..q6, qd1..qd6, qdd1..qdd6, mp, tramo, tau1..tau6 (SI)\n');

csvwrite(fullfile(dir_res, 'ciclo_brazo_tcp.csv'), [C.t', ptcp', zmin']);
fprintf('Figuras: py -3.13 brazo/python/figuras_ciclo.py (lee los CSV de resultados/)\n');
diary off;
