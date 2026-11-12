% ENSAYOS_CONTROL_BRAZO  Comparacion PD + gravedad vs par calculado sobre el ciclo de la tarea
% (enunciado §4.5.4-6 y §7.3). Ensayos:
%   E1 nominal               pieza real = modelo = 0,10 kg
%   E2 carga + perturbacion  pieza real 0,25 kg (modelo 0,10 kg) + pulso de 1 N m en J2, 0,2 s, t = 5 s
%   E3 incertidumbre         masas e inercias reales de los eslabones +15 % respecto del modelo
%   N  error numerico        E1 con par calculado y paso de integracion h = Ts/4
% Metricas: error de posicion y orientacion del TCP (max, RMS, final y durante toma/colocacion),
% par pico por junta, tiempo en saturacion. Registros en resultados/control/.
% Seleccion de ensayos con la variable de entorno AFR_ENSAYOS (por defecto "123N"); p. ej. en PowerShell:
%   $env:AFR_ENSAYOS = "2"; octave-cli ensayos_control_brazo.m
% Cada ensayo guarda su resultado por separado (E<n>_<control>.csv/.mat); el registro se agrega al .txt.
clear; clc; more off;
aqui = fileparts(mfilename('fullpath')); addpath(aqui);
dir_res = fullfile(aqui, '..', 'resultados'); dir_c = fullfile(dir_res, 'control');
if ~exist(dir_c, 'dir'), mkdir(dir_c); end
sel = getenv('AFR_ENSAYOS'); if isempty(sel), sel = '123N'; end
f_log = fullfile(dir_res, 'ensayos_control.txt');
if strcmp(sel, '123N') && exist(f_log, 'file'), delete(f_log); end
diary(f_log);
P = cargar_parametros(); dyn = modelo_dinamico_brazo(P);
fprintf('=== Ensayos de control del brazo (%s) ===\n', datestr(now));
fprintf('wn = %s rad/s, zeta = %.1f, Ts = %g s, friccion conocida por el par calculado = %.0f %%\n', ...
        mat2str(P.brazo.control.wn), P.brazo.control.zeta, P.brazo.control.Ts, 100 * P.brazo.control.friccion_conocida);

f_ref = fullfile(dir_res, 'ciclo_1ms.mat');
if exist(f_ref, 'file')
  S = load(f_ref); C = S.C;
else
  tic; C = generar_ciclo_brazo(P, P.brazo.control.Ts); save(f_ref, 'C'); fprintf('Referencia a 1 ms generada en %.0f s\n', toc);
end
fprintf('Referencia: %.2f s, %d muestras\n\n', C.t(end), numel(C.t));

ensayos = {
  'E1 nominal',               struct('m_real', 0.10);
  'E2 carga 0,25 kg + pert.', struct('m_real', 0.25, 'pert', [2, 5.0, 0.2, 1.0]);
  'E3 masas +15 %',           struct('m_real', 0.10, 'escala_masa_planta', 1.15) };
ctrl = {'pdg', 'ct', 'cti'}; nombre_ctrl = {'PD+g', 'Par calculado', 'PC + integral'};
% Letra 'I' en AFR_ENSAYOS: corre solo la mejora 'cti' en los ensayos 1-3
if any(sel == 'I'), b_lista = 3; sel = [sel '123']; else, b_lista = 1:2; end

% referencia cartesiana (cada 5 ms) para las metricas
paso = 5; idx = 1:paso:numel(C.t);
Tref = zeros(4,4,numel(idx));
for j = 1:numel(idx), Tref(:,:,j) = mcd_brazo(C.q(:,idx(j)), P); end
en_toma = ismember(C.seg(idx), [3 7]);          % esperas de cierre y apertura de la pinza

resumen = {};
fprintf('Ensayo                     Control         e_max  e_RMS  e_fin  e_pinza | ang_max ang_pinza | tau pico [N m] (J1..J3)    sat [%%]  t_sim\n');
fprintf('                                           [mm]   [mm]   [mm]   [mm]    | [deg]   [deg]     |\n');
for a = 1:rows(ensayos)
  if ~any(sel == num2str(a)), continue; end
  for b = b_lista
    op = ensayos{a,2}; op.controlador = ctrl{b};
    tic; R = simular_control_brazo(C, P, dyn, op); ts = toc;
    m = metricas_control(R, C, P, idx, Tref, en_toma);
    fprintf('%-26s %-14s %6.2f %6.2f %6.2f %6.2f   | %6.2f  %6.2f    | %6.2f %6.2f %6.2f     %5.1f   %3.0f s\n', ...
            ensayos{a,1}, nombre_ctrl{b}, m.e_max, m.e_rms, m.e_fin, m.e_pinza, m.a_max, m.a_pinza, ...
            m.tau_pico(1:3), m.sat, ts);
    resumen(end+1, :) = {ensayos{a,1}, nombre_ctrl{b}, m};
    % CSV cada 5 ms: t, q_ref(6), q(6), qdot(6), tau(6), e_pos_mm, e_ang_deg
    csvwrite(fullfile(dir_c, sprintf('E%d_%s.csv', a, ctrl{b})), ...
             [R.t(idx)', C.q(:,idx)', R.q(:,idx)', R.qdot(:,idx)', R.tau(:,idx)', m.ep', m.ea']);
    m.nombre = ensayos{a,1}; m.control = nombre_ctrl{b}; m.t_sim = ts;
    save(fullfile(dir_c, sprintf('E%d_%s.mat', a, ctrl{b})), 'm');
  end
end

% Error numerico: mismo ensayo con paso de integracion 4 veces menor (primeros 3 s del ciclo:
% incluye el traslado inicial y la bajada, con aceleraciones y cambios de tramo)
if any(sel == 'N')
  fprintf('\nComprobacion del error numerico (E1, par calculado, t <= 3 s): h = Ts vs h = Ts/4\n');
  k = find(C.t <= 3.0);
  Cc = C; Cc.t = C.t(k); Cc.q = C.q(:,k); Cc.qd = C.qd(:,k); Cc.qdd = C.qdd(:,k); Cc.mp = C.mp(k); Cc.seg = C.seg(k);
  op = struct('m_real', 0.10, 'controlador', 'ct');
  R1 = simular_control_brazo(Cc, P, dyn, op);
  op.subpasos = 4; R4 = simular_control_brazo(Cc, P, dyn, op);
  fprintf('   max|q(h) - q(h/4)| = %.2e rad ; max|tau(h) - tau(h/4)| = %.2e N m ; max|e(h)| = %.2e rad\n', ...
          max(abs(R1.q(:) - R4.q(:))), max(abs(R1.tau(:) - R4.tau(:))), max(abs(R1.q(:) - Cc.q(:))));
end
diary off;
