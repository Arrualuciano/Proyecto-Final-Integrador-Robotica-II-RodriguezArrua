function correr_agv_desde_gui(ruta_config)
  % CORRER_AGV_DESDE_GUI  Modo virtual del AGV para la interfaz.
  %   octave-cli --eval "correr_agv_desde_gui('<carpeta>/config.json')"
  % config.json: misiones [[x y phi T], ...] en {G} (m, rad, s), modo (0/1/2), carga [kg],
  %   eta_cal [3], kp, ki, kx, ky, kphi, vmax, amax, wmax, alfamax, wrueda_max.
  % Escribe en la carpeta: resultado.csv (cada 10 ms: t, pose real(3), odometria(3), referencia(3),
  %   w_ref(4), w_med(4), u(4)), metricas.json y parametros_usados.json.
  more off;
  aqui = fileparts(mfilename('fullpath')); addpath(aqui);
  cfg = jsondecode(fileread(ruta_config));
  dir_ens = fileparts(ruta_config);
  P = cargar_parametros_agv();
  acot = @(v, a, b) min(max(v, a), b);
  A = agv_parametros(P, acot(cfg.carga, 0, 2));
  f_id = fullfile(aqui, '..', 'resultados', 'identificacion_simulada.mat');
  if exist(f_id, 'file')                       % prealimentacion identificada (como en el robot real)
    I = load(f_id); A.c.K = [I.R.K]'; A.c.udz = [I.R.udz]';
  end
  A.c.kp = acot(cfg.kp, 0, 50); A.c.ki = acot(cfg.ki, 0, 500);
  A.c.kx = acot(cfg.kx, 0, 10); A.c.ky = acot(cfg.ky, 0, 10); A.c.kphi = acot(cfg.kphi, 0, 10);
  A.c.vmax = acot(cfg.vmax, 0.01, 0.5); A.c.amax = acot(cfg.amax, 0.05, 2);
  A.c.wmax = acot(cfg.wmax, 0.1, 4); A.c.alfamax = acot(cfg.alfamax, 0.1, 10);
  A.c.wrueda_max = acot(cfg.wrueda_max, 1, 15);
  A.c.eta_cal = acot(cfg.eta_cal(:), 0.5, 1.5);
  M = cfg.misiones; if isvector(M), M = M(:)'; end
  modo = round(acot(cfg.modo, 0, 2));
  fprintf('Simulando %d mision(es), modo %d, carga %.2f kg...\n', rows(M), modo, A.carga); fflush(stdout);
  tic; S = simular_agv(A, M, struct('modo', modo)); ts = toc;
  csvwrite(fullfile(dir_ens, 'resultado.csv'), [S.t', S.pose', S.od', S.ref', S.wref', S.wmed', S.u']);
  f = S.fin{end}; meta = M(end,1:3)';
  met = struct('error_pos_real_mm', 1000 * norm(f.real(1:2) - meta(1:2)), ...
               'error_ang_real_deg', rad2deg(abs(atan2(sin(f.real(3) - meta(3)), cos(f.real(3) - meta(3))))), ...
               'deriva_odometria_mm', 1000 * norm(f.odom(1:2) - f.real(1:2)), ...
               'pwm_max', max(abs(S.u(:))), 'duracion_s', f.t, 't_simulacion_s', ts, ...
               'pose_real_final', f.real', 'odometria_final', f.odom');
  fid = fopen(fullfile(dir_ens, 'metricas.json'), 'w'); fprintf(fid, '%s', jsonencode(met)); fclose(fid);
  usados = struct('control', A.c, 'carga', A.carga, 'deslizamiento_real', A.eta_des', 'Vbat', A.Vbat, ...
                  'planta', P.agv.planta, 'geometria', struct('r', A.r, 'lx', A.lx, 'ly', A.ly));
  fid = fopen(fullfile(dir_ens, 'parametros_usados.json'), 'w'); fprintf(fid, '%s', jsonencode(usados)); fclose(fid);
  fprintf('LISTO error real %.1f mm / %.2f deg, deriva odometria %.1f mm, %.1f s\n', ...
          met.error_pos_real_mm, met.error_ang_real_deg, met.deriva_odometria_mm, f.t);
end
