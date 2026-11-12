function correr_desde_gui(ruta_config)
  % CORRER_DESDE_GUI  Punto de entrada de la interfaz para simular el brazo (modo virtual).
  %   La interfaz escribe config.json en una carpeta de ensayo y ejecuta:
  %     octave-cli --eval "correr_desde_gui('<carpeta>/config.json')"
  %   Se lee la tabla unica de parametros, se sobrescriben solo los campos del ensayo, se genera
  %   (o se reutiliza) la referencia del ciclo a 1 ms, se simula el lazo cerrado y se escriben en la
  %   misma carpeta: resultado.csv (cada 5 ms), metricas.json y parametros_usados.json.
  %   Columnas de resultado.csv: t, q_ref(6), q(6), qdot(6), tau(6), e_pos_mm, e_ang_deg, tramo.
  more off;
  aqui = fileparts(mfilename('fullpath')); addpath(aqui);
  cfg = jsondecode(fileread(ruta_config));
  dir_ens = fileparts(ruta_config);
  P = cargar_parametros(); dyn = modelo_dinamico_brazo(P);

  % ---- parametros del ensayo (la interfaz ya valido los rangos; aqui se vuelven a acotar)
  acot = @(v, a, b) min(max(v, a), b);
  P.brazo.control.wn = [acot(cfg.wn_123, 5, 60) * ones(3,1); acot(cfg.wn_456, 5, 100) * ones(3,1)];
  P.brazo.control.zeta = acot(cfg.zeta, 0.5, 2);
  P.brazo.control.friccion_conocida = acot(cfg.friccion_conocida, 0, 1);
  margen = acot(cfg.margen_tiempo, 1.0, 2.0);
  op = struct('controlador', cfg.controlador, 'm_real', acot(cfg.m_real, 0, 0.4), ...
              'm_modelo', acot(cfg.m_modelo, 0, 0.4), 'escala_masa_planta', acot(cfg.escala_masa_planta, 0.7, 1.3), ...
              'progreso', true);
  if isfield(cfg, 'pert_activa') && cfg.pert_activa
    op.pert = [round(acot(cfg.pert_junta, 1, 6)), acot(cfg.pert_t0, 0, 30), acot(cfg.pert_dur, 0.01, 2), acot(cfg.pert_amp, -3, 3)];
  end

  % ---- referencia (cacheada por margen de tiempo)
  dir_res = fullfile(aqui, '..', 'resultados');
  f_ref = fullfile(dir_res, sprintf('ciclo_1ms_m%03d.mat', round(100 * margen)));
  if round(100 * margen) == round(100 * P.brazo.tarea.margen_tiempo) && exist(fullfile(dir_res, 'ciclo_1ms.mat'), 'file')
    f_ref = fullfile(dir_res, 'ciclo_1ms.mat');
  end
  if exist(f_ref, 'file')
    S = load(f_ref); C = S.C;
    fprintf('Referencia reutilizada (%s)\n', f_ref);
  else
    fprintf('Generando la referencia del ciclo a 1 ms (margen %.2f), ~1 min...\n', margen); fflush(stdout);
    C = generar_ciclo_brazo(P, P.brazo.control.Ts, margen); save(f_ref, 'C');
  end
  fprintf('Ciclo de %.2f s. Simulando %s...\n', C.t(end), cfg.controlador); fflush(stdout);

  tic; R = simular_control_brazo(C, P, dyn, op); ts = toc;

  paso = 5; idx = 1:paso:numel(C.t);
  Tref = zeros(4,4,numel(idx));
  for j = 1:numel(idx), Tref(:,:,j) = mcd_brazo(C.q(:,idx(j)), P); end
  m = metricas_control(R, C, P, idx, Tref, ismember(C.seg(idx), [3 7]));
  csvwrite(fullfile(dir_ens, 'resultado.csv'), ...
           [R.t(idx)', C.q(:,idx)', R.q(:,idx)', R.qdot(:,idx)', R.tau(:,idx)', m.ep', m.ea', C.seg(idx)']);
  met = struct('e_max_mm', m.e_max, 'e_rms_mm', m.e_rms, 'e_final_mm', m.e_fin, 'e_pinza_mm', m.e_pinza, ...
               'ang_max_deg', m.a_max, 'ang_pinza_deg', m.a_pinza, 'tau_pico_Nm', m.tau_pico, ...
               'tau_limite_Nm', R.tau_lim', 'saturacion_pct', m.sat, 'duracion_ciclo_s', C.t(end), ...
               't_simulacion_s', ts, 'Kp', R.Kp', 'Kd', R.Kd', 'Ki', R.Ki', 'tramos', {C.nombres});
  escribir_json(fullfile(dir_ens, 'metricas.json'), met);
  usados = struct('control', P.brazo.control, 'op', op, 'margen_tiempo', margen, 'pieza', P.pieza, ...
                  'accionamientos', P.brazo.accionamientos, 'friccion', P.brazo.friccion);
  escribir_json(fullfile(dir_ens, 'parametros_usados.json'), usados);
  fprintf('LISTO e_max = %.2f mm, e_pinza = %.2f mm, t_sim = %.0f s\n', m.e_max, m.e_pinza, ts);
end

function escribir_json(ruta, s)
  fid = fopen(ruta, 'w'); fprintf(fid, '%s', jsonencode(s)); fclose(fid);
end
