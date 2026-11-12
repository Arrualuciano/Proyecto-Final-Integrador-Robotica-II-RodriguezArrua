function C = generar_ciclo_brazo(P, dt, margen, mp)
  % GENERAR_CICLO_BRAZO  Referencias del ciclo de la tarea (enunciado §4.5.1-2):
  %   HOME -PTP-> aprox. toma -LIN-> toma | pinza cierra | -LIN-> aprox. toma -PTP-> aprox. colocacion
  %   -LIN-> colocacion (girada 90 grados) | pinza abre | -LIN-> aprox. colocacion -PTP-> HOME
  % Los tramos LIN mantienen la herramienta vertical (orientacion especificada); el giro de 90 grados
  % de la colocacion lo hace el tramo PTP de traslado.
  %   dt     periodo de muestreo de la referencia [s] (por defecto 1 ms, el del controlador)
  %   margen factor sobre la duracion minima (por defecto tarea.margen_tiempo)
  %   mp     masa de la pieza [kg] (por defecto P.pieza.masa)
  % Salida C: t, q, qd, qdd (6xN), mp (1xN, masa en la pinza), seg (1xN), nombres, T_seg, info_lin.
  tar = P.brazo.tarea;
  if nargin < 2 || isempty(dt), dt = 1e-3; end
  if nargin < 3 || isempty(margen), margen = tar.margen_tiempo; end
  if nargin < 4 || isempty(mp), mp = P.pieza.masa; end
  ha = tar.altura_aproximacion;
  R_toma = diag([1 -1 -1]);                                % z_TCP = -z0; dedos cierran segun y0
  g = deg2rad(tar.giro_colocacion_deg);
  R_col = [cos(g) -sin(g) 0; sin(g) cos(g) 0; 0 0 1] * R_toma;
  pT = tar.p_toma(:); pC = tar.p_colocacion(:);
  H = @(R, p) [R, p; 0 0 0 1];
  T_aT = H(R_toma, pT + [0;0;ha]); T_T = H(R_toma, pT);
  T_aC = H(R_col,  pC + [0;0;ha]); T_C = H(R_col,  pC);
  qH = deg2rad(tar.q_home_deg(:));

  C.t = []; C.q = []; C.qd = []; C.qdd = []; C.mp = []; C.seg = [];
  C.nombres = {}; C.T_seg = []; C.info_lin = {};
  q_act = qH;

  function agregar(nombre, t, q, qd, qdd, masa)
    k = numel(C.nombres) + 1;
    C.nombres{k} = nombre; C.T_seg(k) = t(end);
    t0 = 0; if ~isempty(C.t), t0 = C.t(end); t = t(2:end); q = q(:,2:end); qd = qd(:,2:end); qdd = qdd(:,2:end); end
    C.t = [C.t, t0 + t]; C.q = [C.q, q]; C.qd = [C.qd, qd]; C.qdd = [C.qdd, qdd];
    C.mp = [C.mp, masa * ones(1, numel(t))]; C.seg = [C.seg, k * ones(1, numel(t))];
    q_act = q(:,end);
  end
  function espera(nombre, Tw, masa)
    t = 0:dt:Tw; Z = zeros(6, numel(t));
    agregar(nombre, t, repmat(q_act, 1, numel(t)), Z, Z, masa);
  end
  function ptp(nombre, Tdest, masa)
    qB = mci_brazo(Tdest, P, q_act);
    if any(isnan(qB)), error('Pose inalcanzable: %s', nombre); end
    [t, q, qd, qdd] = tramo_ptp(q_act, qB, dt, P, margen);
    agregar(nombre, t, q, qd, qdd, masa);
  end
  function ptp_q(nombre, qB, masa)
    [t, q, qd, qdd] = tramo_ptp(q_act, qB, dt, P, margen);
    agregar(nombre, t, q, qd, qdd, masa);
  end
  function lin(nombre, TA, TB, masa)
    [t, q, qd, qdd, info] = tramo_lin(TA, TB, dt, P, q_act, margen);
    C.info_lin{end+1} = info;
    agregar(nombre, t, q, qd, qdd, masa);
  end

  ptp('PTP HOME -> aprox. toma', T_aT, 0);
  lin('LIN bajada a la toma', T_aT, T_T, 0);
  espera('Pinza cierra', tar.t_pinza, 0);
  lin('LIN subida con pieza', T_T, T_aT, mp);
  ptp('PTP traslado a aprox. colocacion', T_aC, mp);
  lin('LIN bajada a la colocacion', T_aC, T_C, mp);
  espera('Pinza abre', tar.t_pinza, mp);
  lin('LIN subida sin pieza', T_C, T_aC, 0);
  ptp_q('PTP regreso a HOME', qH, 0);
  C.dt = dt; C.margen = margen; C.mp_pieza = mp;
  C.poses = struct('T_aT', T_aT, 'T_T', T_T, 'T_aC', T_aC, 'T_C', T_C, 'qH', qH);
end
