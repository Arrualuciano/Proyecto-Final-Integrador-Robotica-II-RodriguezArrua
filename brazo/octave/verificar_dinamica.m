% VERIFICAR_DINAMICA  Comprobaciones del modelo dinamico del brazo 6R (enunciado §4.4).
%  1) Propiedades de masa: algoritmo de malla vs formula del cubo, y contraste con Fusion.
%  2) Newton-Euler recursivo vs Lagrange (formulaciones independientes).
%  3) M simetrica y definida positiva.
%  4) Caida libre sin par ni friccion: conservacion de la energia (RK4, dos pasos).
%  5) Con pares aplicados: balance de trabajo  E(T) - E(0) = int tau' qd dt.
%  6) Pares de sosten en las poses de la tarea, con y sin pieza.
clear; clc;
aqui = fileparts(mfilename('fullpath')); addpath(aqui);
dir_res = fullfile(aqui, '..', 'resultados'); if ~exist(dir_res, 'dir'), mkdir(dir_res); end
f_log = fullfile(dir_res, 'verificacion_dinamica.txt'); if exist(f_log, 'file'), delete(f_log); end
diary(f_log);
P = cargar_parametros(); rng(2);
fprintf('=== Verificacion dinamica del brazo 6R (%s) ===\n\n', datestr(now));

%% 1) Propiedades de masa
fprintf('1) Propiedades de masa (densidad efectiva %g kg/m3 para la estructura impresa)\n', ...
        P.brazo.material.densidad_efectiva);
tri = stl_leer(fullfile(aqui, '..', 'stl', 'Pieza__estructura.stl')) / 1000;
[m, c, Ic] = propiedades_masa_malla(tri, 1000);
a = P.pieza.lado;
fprintf('   Prueba del algoritmo con el cubo de la pieza (rho = 1000): V = %.3e m3 (exacto %.3e), Ixx = %.4e (exacto m a^2/6 = %.4e)\n', ...
        m / 1000, a^3, Ic(1,1), m * a^2 / 6);
dyn = modelo_dinamico_brazo(P, true);

% contraste con Fusion (volumen y centro de masa de la estructura)
txt = strsplit(fileread(fullfile(aqui, '..', 'cad', 'propiedades_fusion.csv')), {'\r\n', '\n'});
fprintf('   Contraste con Fusion (estructura):  eslabon   V_malla/V_Fusion   |c_malla - c_Fusion| [mm]\n');
for i = 1:6
  Vf = 0; cf = zeros(3,1);                     % suma de los cuerpos 'estructura*' (L2 tiene dos mitades)
  for l = 3:numel(txt)
    col = strsplit(txt{l}, ',');
    if numel(col) > 8 && strcmp(col{1}, dyn.link(i).nombre) && strncmp(col{2}, 'estructura', 10)
      V = str2double(col{5}) * 1e-6; Vf = Vf + V; cf = cf + V * str2double(col(7:9))' / 100;
    end
  end
  if Vf > 0
    ps = dyn.link(i).partes(1);
    fprintf('   %-18s %10.6f %20.4f\n', dyn.link(i).nombre, dyn.link(i).V_estructura / Vf, 1000 * norm(ps.c - cf / Vf));
  end
end

%% 2) Newton-Euler vs Lagrange
fprintf('\n2) Newton-Euler recursivo vs Lagrange (50 estados aleatorios, pieza de %.2f kg)\n', P.brazo.carga_max);
op = struct('carga', P.brazo.carga_max);
eM = 0; ec = 0; eg = 0; et = 0;
for n = 1:50
  q = P.brazo.q_min + rand(6,1) .* (P.brazo.q_max - P.brazo.q_min);
  qd = (2 * rand(6,1) - 1) .* P.brazo.qd_max; qdd = (2 * rand(6,1) - 1) .* P.brazo.qdd_max;
  [Mn, cn, gn] = matrices_dinamicas(q, qd, P, dyn, op);
  [Ml, cl, gl] = lagrange_brazo(q, qd, P, dyn, op);
  tn = rne_brazo(q, qd, qdd, P, dyn, op);
  eM = max(eM, max(abs(Mn(:) - Ml(:)))); ec = max(ec, max(abs(cn - cl)));
  eg = max(eg, max(abs(gn - gl)));       et = max(et, max(abs(tn - (Ml * qdd + cl + gl))));
end
fprintf('   max|M_NE - M_L| = %.2e kg m2 | max|c_NE - c_L| = %.2e N m | max|g_NE - g_L| = %.2e N m | max|tau_NE - tau_L| = %.2e N m\n', ...
        eM, ec, eg, et);

%% 3) Propiedades de M
lmin = inf; asim = 0;
for n = 1:500
  q = P.brazo.q_min + rand(6,1) .* (P.brazo.q_max - P.brazo.q_min);
  Mq = matrices_dinamicas(q, zeros(6,1), P, dyn, op);
  asim = max(asim, max(max(abs(Mq - Mq')))); lmin = min(lmin, min(eig((Mq + Mq') / 2)));
end
fprintf('\n3) M(q) en 500 poses: asimetria max %.1e, autovalor minimo %.3e kg m2 (> 0: definida positiva)\n', asim, lmin);
q0 = zeros(6,1); M0 = matrices_dinamicas(q0, q0, P, dyn, op);
fprintf('   diag(M) en la pose cero [kg m2]: %s  (incluye N^2 Jm = %s)\n', mat2str(diag(M0)', 4), mat2str(dyn.Jref', 3));

%% 4) Energia en caida libre (tau = 0, sin friccion)
fprintf('\n4) Caida libre desde q = (0, 30, 30, 0, 30, 0) deg, tau = 0, 1 s, RK4\n');
f = @(z, tau, opx) [z(7:12); dinamica_directa(z(1:6), z(7:12), tau, P, dyn, opx)];
for hpaso = [2e-3, 1e-3]
  z = [deg2rad([0; 30; 30; 0; 30; 0]); zeros(6,1)];
  E0 = energia_brazo(z(1:6), z(7:12), P, dyn, op); emax = 0;
  for k = 1:round(1 / hpaso)
    k1 = f(z, zeros(6,1), op); k2 = f(z + hpaso/2 * k1, zeros(6,1), op);
    k3 = f(z + hpaso/2 * k2, zeros(6,1), op); k4 = f(z + hpaso * k3, zeros(6,1), op);
    z = z + hpaso / 6 * (k1 + 2*k2 + 2*k3 + k4);
    if mod(k, 10) == 0, emax = max(emax, abs(energia_brazo(z(1:6), z(7:12), P, dyn, op) - E0)); end
  end
  fprintf('   h = %.0e s: E0 = %.4f J, error maximo de energia = %.2e J (%.1e %%)\n', hpaso, E0, emax, 100 * emax / abs(E0));
end

%% 5) Balance de trabajo con pares aplicados
fprintf('\n5) Balance de trabajo: tau(t) = g(q) + 0,3 sin(2 pi t) (1,1,1,0.1,0.1,0.1) N m, 1 s, h = 1 ms\n');
hpaso = 1e-3; z = [deg2rad([0; 30; 30; 0; 30; 0]); zeros(6,1)]; W = 0;
E0 = energia_brazo(z(1:6), z(7:12), P, dyn, op);
taufun = @(t, z) rne_brazo(z(1:6), zeros(6,1), zeros(6,1), P, dyn, op) + 0.3 * sin(2*pi*t) * [1;1;1;0.1;0.1;0.1];
for k = 0:round(1 / hpaso) - 1
  t = k * hpaso;
  g1 = @(tt, zz) [zz(7:12); dinamica_directa(zz(1:6), zz(7:12), taufun(tt, zz), P, dyn, op)];
  k1 = g1(t, z); k2 = g1(t + hpaso/2, z + hpaso/2 * k1); k3 = g1(t + hpaso/2, z + hpaso/2 * k2); k4 = g1(t + hpaso, z + hpaso * k3);
  p1 = taufun(t, z)' * z(7:12);
  zn = z + hpaso / 6 * (k1 + 2*k2 + 2*k3 + k4);
  p2 = taufun(t + hpaso, zn)' * zn(7:12);
  W = W + hpaso / 2 * (p1 + p2); z = zn;           % trapecios
end
E1 = energia_brazo(z(1:6), z(7:12), P, dyn, op);
fprintf('   E(1) - E(0) = %.5f J ; int tau^T qd dt = %.5f J ; diferencia %.1e J\n', E1 - E0, W, abs(E1 - E0 - W));

%% 6) Pares de sosten en las poses de la tarea
fprintf('\n6) Pares de sosten g(q) en las poses de la tarea [N m]\n');
S = load(fullfile(dir_res, 'cinematica_tarea.mat'));
for k = 1:size(S.Q_tarea, 2)
  g_sin = rne_brazo(S.Q_tarea(:,k), zeros(6,1), zeros(6,1), P, dyn, struct('carga', 0));
  g_con = rne_brazo(S.Q_tarea(:,k), zeros(6,1), zeros(6,1), P, dyn, struct('carga', P.pieza.masa));
  fprintf('   %-18s sin pieza %s | con pieza %s\n', S.poses{k,1}, mat2str(g_sin', 3), mat2str(g_con', 3));
end
save(fullfile(dir_res, 'dinamica_parametros.mat'), 'dyn');
diary off;
