% VERIFICAR_CINEMATICA  Comprobaciones de MCD, MCI, Jacobiano, singularidades y estatica
% del brazo 6R (enunciado §4.3 y §7.3). Guarda el registro en brazo/resultados/.
clear; clc;
aqui = fileparts(mfilename('fullpath')); addpath(aqui);
dir_res = fullfile(aqui, '..', 'resultados'); if ~exist(dir_res, 'dir'), mkdir(dir_res); end
diary_file = fullfile(dir_res, 'verificacion_cinematica.txt');
if exist(diary_file, 'file'), delete(diary_file); end
diary(diary_file);
P = cargar_parametros();
M = P.brazo.dh_mat; d1 = M(1,2); a2 = M(2,3); d4 = M(4,2); d6 = M(6,2); h = P.brazo.herramienta.tcp_z6;
rng(1);   % semilla fija: resultados reproducibles
fprintf('=== Verificacion cinematica del brazo 6R  (%s) ===\n', datestr(now));
fprintf('DH: d1=%.3f a2=%.3f d4=%.3f d6=%.3f herramienta=%.3f m\n\n', d1, a2, d4, d6, h);

%% 1) MCD en configuraciones conocidas (resultado esperado deducido del dibujo)
fprintf('1) MCD en configuraciones conocidas\n');
casos = { [0 0 0 0 0 0],      [d4+d6+h; 0; d1+a2],      'pose cero: brazo vertical, antebrazo y herramienta hacia +x';
          [90 0 0 0 0 0],     [0; d4+d6+h; d1+a2],      'q1 = 90: el mismo brazo girado hacia +y';
          [0 0 0 0 90 0],     [d4; 0; d1+a2-d6-h],      'q5 = 90: herramienta hacia abajo';
          [0 90 -90 0 0 0],   [a2+d4+d6+h; 0; d1],      'q2 = 90, q3 = -90: brazo estirado horizontal (singular)';
          [0 0 90 0 0 0],     [0; 0; d1+a2-d4-d6-h],    'q3 = 90: antebrazo hacia abajo' };
for k = 1:rows(casos)
  Tt = mcd_brazo(deg2rad(casos{k,1}(:)), P);
  e = norm(Tt(1:3,4) - casos{k,2});
  fprintf('   %-58s TCP = (%7.4f %7.4f %7.4f)  error = %.1e m\n', casos{k,3}, Tt(1:3,4), e);
end

%% 2) MCI: ida y vuelta con configuraciones aleatorias dentro de limites
fprintf('\n2) MCI analitica (8 soluciones) - prueba de ida y vuelta\n');
N = 2000; err_max = 0; encontrada = 0; nsol = zeros(1, N);
for n = 1:N
  q0 = P.brazo.q_min + rand(6,1) .* (P.brazo.q_max - P.brazo.q_min);
  T0 = mcd_brazo(q0, P);
  [~, Q] = mci_brazo(T0, P, q0);
  ok = ~any(isnan(Q));
  nsol(n) = sum(ok);
  for k = find(ok)
    Tk = mcd_brazo(Q(:,k), P);
    err_max = max(err_max, norm(Tk - T0, 'fro'));
  end
  d = min(vecnorm(atan2(sin(Q(:,ok) - q0), cos(Q(:,ok) - q0))));
  encontrada += (d < 1e-6);
end
fprintf('   %d poses: error maximo MCD(MCI(T)) - T = %.2e (norma de Frobenius)\n', N, err_max);
fprintf('   La configuracion original esta entre las soluciones en %d de %d casos\n', encontrada, N);
fprintf('   Soluciones geometricas por pose: min %d, max %d (8 = hombro x codo x muneca)\n', min(nsol), max(nsol));

%% 3) Jacobiano geometrico frente a diferencias finitas
fprintf('\n3) Jacobiano geometrico vs derivada numerica de la MCD\n');
hd = 1e-6; e_J = 0; ratio = [];
for n = 1:200
  q = P.brazo.q_min + rand(6,1) .* (P.brazo.q_max - P.brazo.q_min);
  [J, inf_j] = jacobiano_brazo(q, P);
  Jn = zeros(6);
  for i = 1:6
    dq = zeros(6,1); dq(i) = hd;
    Tp = mcd_brazo(q + dq, P); Tm = mcd_brazo(q - dq, P);
    Jn(1:3, i) = (Tp(1:3,4) - Tm(1:3,4)) / (2*hd);
    dR = Tp(1:3,1:3) * Tm(1:3,1:3)';                % rotacion incremental en {0}
    Jn(4:6, i) = [dR(3,2) - dR(2,3); dR(1,3) - dR(3,1); dR(2,1) - dR(1,2)] / 2 / (2*hd);
  end
  e_J = max(e_J, max(abs(J(:) - Jn(:))));
  if abs(inf_j.det_formula) > 1e-6, ratio(end+1) = inf_j.det / inf_j.det_formula; end
end
fprintf('   Diferencia maxima J - J_numerico = %.2e\n', e_J);
fprintf('   det(J) / [a2 d4 rho cos(q3) sin(q5)] = %.6f (desvio %.1e)  => det(J) = %+.0f a2 d4 rho cos(q3) sin(q5)\n', ...
        mean(ratio), std(ratio), mean(ratio));
fprintf('   Singularidades: hombro rho = 0 (muneca sobre el eje 1); codo cos q3 = 0 (brazo estirado o plegado);\n');
fprintf('                   muneca sin q5 = 0 (ejes 4 y 6 alineados)\n');

%% 4) Poses de la tarea (00_Aplicacion_y_plan.md, seccion 3)
fprintf('\n4) Poses de la tarea con herramienta hacia abajo\n');
R_toma = diag([1 -1 -1]);                          % z_TCP = -z0, dedos cierran segun y0
Rz90 = [0 -1 0; 1 0 0; 0 0 1];
poses = { 'aprox. toma',     [0.30; 0; 0.10], R_toma;
          'toma',            [0.30; 0; 0.02], R_toma;
          'aprox. colocacion',[0; 0.30; 0.10], Rz90 * R_toma;
          'colocacion',      [0; 0.30; 0.02], Rz90 * R_toma };
q_ref = deg2rad([0; 30; 45; 0; 60; 0]);           % referencia: postura de trabajo frente al robot
Q_tarea = zeros(6, rows(poses));
for k = 1:rows(poses)
  Td = eye(4); Td(1:3,1:3) = poses{k,3}; Td(1:3,4) = poses{k,2};
  [q, Q, inf_k] = mci_brazo(Td, P, q_ref);
  Tq = mcd_brazo(q, P);
  [J, inf_j] = jacobiano_brazo(q, P);
  c = inf_k.config(:, inf_k.elegida);
  fprintf('   %-18s q = [%7.2f %7.2f %7.2f %7.2f %7.2f %7.2f] deg | error %.1e m | config (h,c,m) = (%+d,%+d,%+d) | %d sol. validas\n', ...
          poses{k,1}, rad2deg(q), norm(Tq(1:3,4) - poses{k,2}), c, sum(inf_k.valida));
  fprintf('   %-18s rho = %.3f m, cos q3 = %.3f, sin q5 = %.3f, |det J| = %.2e\n', '', inf_j.factores, abs(inf_j.det));
  Q_tarea(:, k) = q; q_ref = q;
end

%% 5) Continuidad de la MCI en un tramo cartesiano (descenso vertical de 80 mm a la toma)
fprintf('\n5) Continuidad en el descenso LIN aproximacion -> toma (100 muestras)\n');
q_prev = Q_tarea(:, 1); salto = 0; e_lin = 0;
for z = linspace(0.10, 0.02, 100)
  Td = eye(4); Td(1:3,1:3) = R_toma; Td(1:3,4) = [0.30; 0; z];
  q = mci_brazo(Td, P, q_prev);
  salto = max(salto, max(abs(q - q_prev)));
  Tq = mcd_brazo(q, P); e_lin = max(e_lin, norm(Tq(1:3,4) - Td(1:3,4)));
  q_prev = q;
end
fprintf('   Mayor cambio articular entre muestras: %.3f deg (sin saltos de rama); error de posicion max %.1e m\n', ...
        rad2deg(salto), e_lin);

%% 6) Velocidades: Jacobiano inverso en la toma
fprintf('\n6) Jacobiano inverso: descenso a 0,05 m/s sin girar la herramienta, en la pose de toma\n');
J = jacobiano_brazo(Q_tarea(:,2), P);
qd = J \ [0; 0; -0.05; 0; 0; 0];
fprintf('   qd = [%7.3f %7.3f %7.3f %7.3f %7.3f %7.3f] deg/s ; comprobacion |J qd - v| = %.1e\n', ...
        rad2deg(qd), norm(J*qd - [0;0;-0.05;0;0;0]));

%% 7) Estatica: tau = J^T F (convencion de la catedra: tau_motor = g(q) - J^T F_ext)
fprintf('\n7) Estatica en la colocacion (solo cargas externas; el peso propio va en dinamica)\n');
qc = Q_tarea(:, 4);
J = jacobiano_brazo(qc, P);
g0 = 9.81; mp = P.pieza.masa;
F_peso = [0; 0; -mp*g0; 0; 0; 0];                 % peso de la pieza, aplicado en el TCP (ambiente -> robot)
F_ins  = [0; 0; 5; 0; 0; 0];                      % reaccion de la bandeja al insertar empujando 5 N hacia abajo
for caso = {'peso de la pieza', F_peso; 'peso + insercion 5 N', F_peso + F_ins}'
  tau = -J' * caso{2};
  fprintf('   %-22s tau_motor = [%7.4f %7.4f %7.4f %7.4f %7.4f %7.4f] N m\n', caso{1}, tau);
end
fprintf('   Comprobacion por potencia: F^T (J qd) = tau_ext^T qd para qd aleatorio: %.1e\n', ...
        abs((F_peso' * (J * (qd = randn(6,1)))) - (J' * F_peso)' * qd));

save(fullfile(dir_res, 'cinematica_tarea.mat'), 'Q_tarea', 'poses');
diary off;
fprintf('\nRegistro guardado en %s\n', diary_file);
