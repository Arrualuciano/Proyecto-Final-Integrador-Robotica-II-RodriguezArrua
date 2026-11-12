function R = simular_control_brazo(C, P, dyn, op)
  % SIMULAR_CONTROL_BRAZO  Planta dinamica del brazo bajo un controlador muestreado (Tema V).
  %
  % PLANTA (integrada aca, en Octave): M(q) qdd + c + g + b qd + tau_C tanh(qd/eps) = tau + d(t)
  %   - masas reales: escala_masa_planta x CAD; pieza real m_real en los tramos con pieza;
  %   - par aplicado tau = sat(tau_cmd, +-eta N tau_m,max)  (limite fisico de cada accionamiento);
  %   - integracion: Euler semi-implicito (qd += h qdd; q += h qd) con 'subpasos' por periodo.
  % CONTROLADOR (cada Ts, retencion de orden cero, con el MODELO nominal):
  %   'pdg': tau = Kp e + Kd ed + g^(q),        Kp_i = M_ii(q_home) wn_i^2, Kd_i = 2 zeta wn_i M_ii(q_home)
  %   'ct' : tau = M^(q) (qdd_d + Kd ed + Kp e) + h^(q,qd) + f^(qd),  Kp = wn^2, Kd = 2 zeta wn
  %          (f^ = fraccion 'friccion_conocida' de la friccion real: error de modelo deliberado)
  %   Ambas apuntan a la misma dinamica del error, e = qd - q (Tema V §5.3 y §5.7).
  %   'cti': mejora propuesta, par calculado con accion integral:
  %          a_c = qdd_d + Kd ed + Kp e + Ki int(e),  Kd = 3 wn, Kp = 3 wn^2, Ki = wn^3 (triple polo en -wn),
  %          con integracion condicionada (no integra si la junta satura y el error empuja hacia la saturacion).
  % op: controlador, m_real, m_modelo, escala_masa_planta, pert = [junta t0 dur amp], subpasos
  % R: t (cada Ts), q, qdot, tau, tau_cmd, sat (6xN logico)
  def = struct('controlador', 'ct', 'm_real', P.pieza.masa, 'm_modelo', P.pieza.masa, ...
               'escala_masa_planta', 1, 'pert', [], 'subpasos', 1);
  f = fieldnames(def); for k = 1:numel(f), if ~isfield(op, f{k}), op.(f{k}) = def.(f{k}); end, end
  cc = P.brazo.control;
  Ts = cc.Ts; h = Ts / op.subpasos; wn = cc.wn(:); z = cc.zeta; eps_c = cc.eps_coulomb;

  % planta y modelo, sin y con pieza
  Dp0 = preparar_dinamica(P, dyn, 0, op.escala_masa_planta);
  Dp1 = preparar_dinamica(P, dyn, op.m_real, op.escala_masa_planta);
  Dm0 = preparar_dinamica(P, dyn, 0);
  Dm1 = preparar_dinamica(P, dyn, op.m_modelo);
  tau_lim = Dm0.eta .* Dm0.N .* Dm0.tau_motor_max;

  % ganancias
  Mh = dinamica_rapida(C.q(:,1), zeros(6,1), Dm0);
  Ki = zeros(6,1); integ = zeros(6,1);
  if strcmp(op.controlador, 'pdg')
    Kp = diag(Mh) .* wn.^2; Kd = 2 * z * wn .* diag(Mh);
  elseif strcmp(op.controlador, 'cti')
    Kp = 3 * wn.^2; Kd = 3 * wn; Ki = wn.^3;
  else
    Kp = wn.^2; Kd = 2 * z * wn;
  end

  % la referencia C debe estar muestreada con el periodo del controlador
  if abs(C.dt - Ts) > 1e-12, error('La referencia debe tener dt = Ts'); end
  N = numel(C.t);
  q = C.q(:,1); qd = C.qd(:,1);
  R.t = C.t; R.q = zeros(6,N); R.qdot = zeros(6,N); R.tau = zeros(6,N); R.tau_cmd = zeros(6,N);
  R.sat = false(6,N);
  progreso = isfield(op, 'progreso') && op.progreso; prox = 0.1;
  for k = 1:N
    if progreso && k / N >= prox
      fprintf('PROGRESO %d\n', round(100 * prox)); fflush(stdout); prox = prox + 0.1;
    end
    R.q(:,k) = q; R.qdot(:,k) = qd;
    e = C.q(:,k) - q; ed = C.qd(:,k) - qd;
    con_pieza = C.mp(k) > 0;
    if con_pieza, Dm = Dm1; else, Dm = Dm0; end
    if strcmp(op.controlador, 'pdg')
      [~, gh] = dinamica_rapida(q, zeros(6,1), Dm);
      tc = Kp .* e + Kd .* ed + gh;
    else
      [Mm, hm] = dinamica_rapida(q, qd, Dm);
      fh = cc.friccion_conocida * (Dm.b .* qd + Dm.tau_c .* tanh(qd / eps_c));
      tc = Mm * (C.qdd(:,k) + Kd .* ed + Kp .* e + Ki .* integ) + hm + fh;
    end
    ta = min(max(tc, -tau_lim), tau_lim);
    if any(Ki)                                        % antiwindup por integracion condicionada (Tema V §5.4)
      bloquear = (tc > tau_lim & e > 0) | (tc < -tau_lim & e < 0);
      integ = integ + Ts * e .* ~bloquear;
    end
    R.tau_cmd(:,k) = tc; R.tau(:,k) = ta; R.sat(:,k) = abs(tc) > tau_lim;
    % perturbacion de par externa
    d = zeros(6,1);
    if ~isempty(op.pert)
      t = C.t(k);
      if t >= op.pert(2) && t < op.pert(2) + op.pert(3), d(op.pert(1)) = op.pert(4); end
    end
    if con_pieza, Dp = Dp1; else, Dp = Dp0; end
    for s = 1:op.subpasos
      [Mp, hp] = dinamica_rapida(q, qd, Dp);
      fr = Dp.b .* qd + Dp.tau_c .* tanh(qd / eps_c);
      qdd = Mp \ (ta + d - hp - fr);
      qd = qd + h * qdd;
      q = q + h * qd;
    end
  end
  R.op = op; R.Kp = Kp; R.Kd = Kd; R.Ki = Ki; R.tau_lim = tau_lim;
end
