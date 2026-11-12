% ENSAYOS_AGV  Ensayos simulados del AGV con los mismos nombres que los fisicos (enunciado §5.4-5.5):
%   T1 avance longitudinal 1,0 m | T2 desplazamiento lateral 0,6 m | T3 giro de 90 grados sobre el centro
%   T4 trayecto entre estaciones A -> B (1,2 m en x y 0,6 m lateral) | T5 = T4 con 0,5 kg adicionales
% Estrategias (mismas condiciones): 0 lazo abierto (prealimentacion), 1 PI de ruedas, 2 PI + lazo de pose
% con odometria nominal, 2C con odometria calibrada (factores eta obtenidos de T1-T3 como en el laboratorio).
% El controlador usa el modelo identificado con las ruedas elevadas (identificacion_simulada.m).
% Metricas: error final real respecto de la meta, deriva de la odometria (|odometria - real|), PWM maximo.
clear; clc; more off;
aqui = fileparts(mfilename('fullpath')); addpath(aqui);
dir_res = fullfile(aqui, '..', 'resultados'); dir_e = fullfile(dir_res, 'ensayos');
if ~exist(dir_e, 'dir'), mkdir(dir_e); end
f_log = fullfile(dir_res, 'ensayos_agv.txt'); if exist(f_log, 'file'), delete(f_log); end
diary(f_log);
P = cargar_parametros_agv();
fprintf('=== Ensayos simulados del AGV (%s) ===\n', datestr(now));

% modelo de prealimentacion: el MISMO que tiene cargado el robot (11/11/2026): K identificado con las ruedas
% elevadas y zona muerta medida en el piso (parametros.json, agv.identificado). Antes se usaba el resultado de
% identificacion_simulada.m (planta supuesta).
Kid = P.agv.identificado.K(:); udzid = P.agv.identificado.udz_piso(:);
fprintf('Prealimentacion del robot: K = %s rad/s/PWM, u_dz (piso) = %s PWM\n', mat2str(Kid', 4), mat2str(udzid', 3));
A0 = agv_parametros(P, 0); A0.c.K = Kid; A0.c.udz = udzid;
fprintf('Planta: deslizamiento real eta = %s (x, y, giro); Vbat = %.1f V; motores con dispersion %s\n\n', ...
        mat2str(A0.eta_des', 3), A0.Vbat, mat2str(P.agv.planta.dispersion_motores, 3));

T = { 'T1 avance 1,0 m',      [1.0 0 0 0];
      'T2 lateral 0,6 m',     [0 0.6 0 0];
      'T3 giro 90 grados',    [0 0 pi/2 0];
      'T4 A -> B',            [1.2 0 0 0; 1.2 0.6 0 0];
      'T5 A -> B con 0,5 kg', [1.2 0 0 0; 1.2 0.6 0 0] };
carga = [0 0 0 0 0.5];

% ---- calibracion de la odometria con T1-T3 en modo 2 (medicion "externa" = pose real)
eta = [1; 1; 1];
for k = 1:3
  S = simular_agv(A0, T{k,2}, struct('modo', 2));
  f = S.fin{end};
  if k == 1, eta(1) = f.real(1) / f.odom(1); end
  if k == 2, eta(2) = f.real(2) / f.odom(2); end
  if k == 3, eta(3) = f.real(3) / f.odom(3); end
end
fprintf('Calibracion (real / odometria en T1, T2, T3): eta_x = %.4f, eta_y = %.4f, eta_w = %.4f  (verdaderos %s)\n\n', eta, mat2str(A0.eta_des', 3));

estr = {'0 abierto', 0, [1;1;1]; '1 PI ruedas', 1, [1;1;1]; '2 PI + pose', 2, [1;1;1]; '2C pose + odom. calibrada', 2, eta};
fprintf('%-22s %-26s %9s %9s %10s %8s %7s\n', 'Ensayo', 'Estrategia', 'e_pos[mm]', 'e_ang[deg]', 'deriva[mm]', 'PWMmax', 'T[s]');
res = {};
for k = 1:rows(T)
  for s = 1:rows(estr)
    A = agv_parametros(P, carga(k)); A.c.K = Kid; A.c.udz = udzid; A.c.eta_cal = estr{s,3};
    S = simular_agv(A, T{k,2}, struct('modo', estr{s,2}));
    f = S.fin{end}; meta = T{k,2}(end,1:3)';
    ep = 1000 * norm(f.real(1:2) - meta(1:2));
    ea = rad2deg(abs(atan2(sin(f.real(3) - meta(3)), cos(f.real(3) - meta(3)))));
    dr = 1000 * norm(f.odom(1:2) - f.real(1:2));
    fprintf('%-22s %-26s %9.1f %9.2f %10.1f %8.0f %7.1f\n', T{k,1}, estr{s,1}, ep, ea, dr, max(abs(S.u(:))), f.t);
    res(end+1,:) = {T{k,1}, estr{s,1}, ep, ea, dr, max(abs(S.u(:))), f.t};
    csvwrite(fullfile(dir_e, sprintf('T%d_E%d.csv', k, s)), [S.t', S.pose', S.od', S.ref', S.wref', S.wmed', S.u']);
  end
end
save(fullfile(dir_res, 'ensayos_agv.mat'), 'res', 'eta');
fprintf('\nCSV: t, pose real (x y phi), odometria (x y phi), referencia (x y phi), w_ref(4), w_med(4), u(4)\n');
diary off;
