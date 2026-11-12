function S = simular_agv(A, misiones, op)
  % SIMULAR_AGV  AGV mecanum en lazo cerrado: planta dinamica + mismo algoritmo que el firmware.
  %
  % PLANTA (integrada aca, h = 1 ms, Euler semi-implicito), estado vb = (vx, vy, w) en el cuerpo:
  %   Mb vb_dot = H' tau + [m w vy; -m w vx; 0]               (Newton-Euler plano con rodadura w_r = H vb)
  %   Mb = diag(m, m, Iz) + Jw H'H  (Jw = N^2 J_rotor + J_rueda: inercia reflejada de rotor y rueda)
  %   tau_i = Kt_i (V_i - Ke w_i)/R - b w_i - (tau_c,red + tau_c,rod) tanh(w_i/eps),  V_i = u_i/255 Vbat
  %   Movimiento sobre el piso con deslizamiento de rodillos: v_suelo = diag(eta_des) vb
  %   (la odometria no lo ve). Encoders: angulo de cada rueda cuantizado a cpr cuentas por vuelta.
  % CONTROLADOR (copia de agv/firmware/agv_osoyoo/nucleo.cpp), Ts = 10 ms, pose cada 20 ms:
  %   odometria con rumbo en el punto medio y factores eta_cal; quintico en (x, y, phi);
  %   modo 2: V = R(phi)' (xi_d' + K (xi_d - xi)); modo 1: V = R(phi_d)' xi_d'; modo 0: igual a 1 pero
  %   las ruedas solo con prealimentacion u = sgn(w)(|w|/K + u_dz) (lazo abierto);
  %   PI de rueda con prealimentacion e integracion condicionada; desaturacion por escala comun.
  % misiones: filas [x y phi T] (en {G}); se ejecutan en orden, cada una desde el reposo.
  % op: modo (0/1/2), t_extra (s de reposo al final), elevado (1 = ruedas en el aire: sin cuerpo)
  % S: registro cada 10 ms (t, pose real, odometria, referencia, w_ref, w_med, u) y eventos de fin.
  if nargin < 3, op = struct(); end
  if ~isfield(op, 'modo'), op.modo = 2; end
  if ~isfield(op, 't_extra'), op.t_extra = 1.0; end
  if ~isfield(op, 'elevado'), op.elevado = 0; end
  if ~isfield(op, 'pasos_pwm'), op.pasos_pwm = []; end   % para identificacion: filas [t0 dur u]
  c = A.c; h = 1e-3; nTs = round(c.Ts / h); H = A.H; L = A.L;
  tc = A.tc_red + (1 - op.elevado) * A.tc_rod;

  % ---- estado de la planta
  pose = [0; 0; 0]; vb = zeros(3,1); wr = zeros(4,1); ang = zeros(4,1);
  % ---- estado del controlador (firmware)
  od = [0; 0; 0]; cnt_prev = zeros(4,1); w_med = zeros(4,1); integ = zeros(4,1); u = zeros(4,1);
  vbc = zeros(3,1); k_pose = 0; mis = 1; tr = []; fin = {}; activo = ~isempty(misiones);
  if activo, tr = nueva(od, misiones(1,:), c); end
  % ---- duracion total aproximada
  if ~isempty(op.pasos_pwm)
    Ttot = max(op.pasos_pwm(:,1) + op.pasos_pwm(:,2)) + op.t_extra;
  else
    Ttot = 0; o = od;
    for k = 1:rows(misiones), q = nueva(o, misiones(k,:), c); Ttot = Ttot + q.T + 2.5; o = misiones(k,1:3)'; end
    Ttot = Ttot + op.t_extra;
  end
  Nst = ceil(Ttot / h); Nlog = floor(Nst / nTs);
  S.t = zeros(1,Nlog); S.pose = zeros(3,Nlog); S.od = zeros(3,Nlog); S.ref = zeros(3,Nlog);
  S.wref = zeros(4,Nlog); S.wmed = zeros(4,Nlog); S.wreal = zeros(4,Nlog); S.u = zeros(4,Nlog); S.vb = zeros(3,Nlog);
  il = 0; ref = od;
  for n = 0:Nst-1
    t = n * h;
    if mod(n, nTs) == 0
      % ================= controlador: medicion de encoders
      cnt = floor(ang * A.cpr / (2*pi));
      dphi = 2*pi * (cnt - cnt_prev) / A.cpr; cnt_prev = cnt;
      w_med = c.alfa_vel * dphi / c.Ts + (1 - c.alfa_vel) * w_med;
      ds = A.r * dphi;
      d = A.Hp * (dphi);  d = d .* c.eta_cal;           % (dx, dy, dth) del cuerpo
      hh = od(3) + d(3)/2;
      od = od + [d(1)*cos(hh) - d(2)*sin(hh); d(1)*sin(hh) + d(2)*cos(hh); d(3)];
      if ~isempty(op.pasos_pwm)                          % ensayo de escalones (identificacion)
        u = zeros(4,1);
        for k = 1:rows(op.pasos_pwm)
          if t >= op.pasos_pwm(k,1) && t < op.pasos_pwm(k,1) + op.pasos_pwm(k,2), u(:) = op.pasos_pwm(k,3); end
        end
        wref = zeros(4,1);
      elseif activo
        % ================= trayectoria y lazo de pose (cada 20 ms)
        tr.t = tr.t + c.Ts; k_pose = k_pose + 1;
        if k_pose >= c.div_pose || tr.t <= c.Ts + 1e-9
          k_pose = 0;
          uu = min(max(tr.t / tr.T, 0), 1);
          sp = uu^3 * (10 - 15*uu + 6*uu^2); sv = (uu < 1) * (30*uu^2 - 60*uu^3 + 30*uu^4) / tr.T;
          ref = tr.x0 + tr.d * sp; Vw = tr.d * sv; e = [0;0;0]; phu = ref(3);
          if op.modo == 2
            e = ref - od; e(3) = atan2(sin(e(3)), cos(e(3)));
            Vw = Vw + [c.kx; c.ky; c.kphi] .* e; phu = od(3);
          end
          vbc = [cos(phu)*Vw(1) + sin(phu)*Vw(2); -sin(phu)*Vw(1) + cos(phu)*Vw(2); Vw(3)];
          v = norm(vbc(1:2)); vl = 1.3 * c.vmax; if v > vl, vbc(1:2) = vbc(1:2) * vl / v; end
          vbc(3) = min(max(vbc(3), -1.3*c.wmax), 1.3*c.wmax);
          if uu >= 1 && (op.modo < 2 || (norm(e(1:2)) < 0.005 && abs(e(3)) < 0.01) || tr.t > tr.T + 2)
            fin{end+1} = struct('t', t, 'mision', mis, 'meta', misiones(mis,1:3)', 'real', pose, 'odom', od);
            vbc = zeros(3,1); integ(:) = 0;
            mis = mis + 1;
            if mis <= rows(misiones), tr = nueva(od, misiones(mis,:), c); else, activo = false; end
          end
        end
        % ================= ruedas
        wref = H * vbc; m = max(abs(wref)); if m > c.wrueda_max, wref = wref * c.wrueda_max / m; end
        for i = 1:4
          ff = 0; if abs(wref(i)) > 0.05, ff = sign(wref(i)) * (abs(wref(i)) / c.K(i) + c.udz(i)); end
          if op.modo == 0
            u(i) = min(max(ff, -255), 255); integ(i) = 0;
          elseif abs(wref(i)) < 0.05 && abs(w_med(i)) < 0.3
            u(i) = 0; integ(i) = 0;
          else
            e = wref(i) - w_med(i);
            ut = ff + c.kp * e + c.ki * (integ(i) + e * c.Ts);
            if abs(ut) <= 255 || sign(ut) ~= sign(e), integ(i) = integ(i) + e * c.Ts; end
            u(i) = min(max(ff + c.kp * e + c.ki * integ(i), -255), 255);
          end
        end
        u = fix(u);                                         % el firmware envia PWM entero
      else
        wref = zeros(4,1); u = zeros(4,1); integ(:) = 0;
      end
      il = il + 1;
      if il <= Nlog
        S.t(il) = t; S.pose(:,il) = pose; S.od(:,il) = od; S.ref(:,il) = ref; S.wref(:,il) = wref;
        S.wmed(:,il) = w_med; S.wreal(:,il) = wr; S.u(:,il) = u; S.vb(:,il) = vb;
      end
    end
    % ================= planta (1 ms)
    V = u / 255 * A.Vbat;
    tau = A.Kt .* (V - A.Ke * wr) / A.R - A.b * wr - tc * tanh(wr / A.eps);
    if op.elevado
      wr = wr + h * tau / A.Jw;                             % ruedas libres: solo inercia propia
    else
      rhs = H' * tau + [A.m * vb(3) * vb(2); -A.m * vb(3) * vb(1); 0];
      vb = vb + h * (A.Mb_inv * rhs);
      wr = H * vb;
      vg = A.eta_des .* vb;                                 % deslizamiento de rodillos
      hh = pose(3) + vg(3) * h / 2;
      pose = pose + h * [vg(1)*cos(hh) - vg(2)*sin(hh); vg(1)*sin(hh) + vg(2)*cos(hh); vg(3)];
    end
    ang = ang + h * wr;
  end
  S.fin = fin; S.misiones = misiones; S.op = op;
end

function tr = nueva(od, mis, c)
  % Quintico desde la odometria actual hasta la meta, con duracion minima por limites (Tema IV §4.6)
  tr.x0 = od; tr.d = [mis(1) - od(1); mis(2) - od(2); atan2(sin(mis(3) - od(3)), cos(mis(3) - od(3)))];
  d = norm(tr.d(1:2)); a = abs(tr.d(3));
  Tm = max([1.875 * d / c.vmax, sqrt(5.7735 * d / c.amax), 1.875 * a / c.wmax, sqrt(5.7735 * a / c.alfamax), 0.5]);
  tr.T = max(mis(4), Tm); tr.t = 0;
end
