function [t, q, qd, qdd, info] = tramo_lin(TA, TB, dt, P, q0, margen)
  % TRAMO_LIN  Tramo cartesiano: recta del TCP + interpolacion de orientacion por eje-angulo (Tema IV §4.8-4.9).
  %   p(s) = pA + (pB - pA) s ;  R(s) = RA * Rot(n, s*theta), con RA' RB = Rot(n, theta) (Rodrigues)
  %   s(t) quintico. Duracion minima por v_cart_max, a_cart_max (traslacion) y qd_max/qdd_max de la muneca
  %   (giro); luego se alarga en 5 % hasta que TODAS las juntas respetan qd_max y qdd_max.
  %   Juntas: q por MCI con la solucion continua (referencia = muestra anterior);
  %   qd = J \ [pd; w],  qdd = J \ ([pdd; alfa] - Jdot qd),  Jdot por diferencia central en la direccion de qd.
  %   info: error maximo de la MCD contra la recta pedida, minimo |det J|, numero de alargues, T.
  if nargin < 6 || isempty(margen), margen = 1; end
  pA = TA(1:3,4); pB = TB(1:3,4); RA = TA(1:3,1:3); RB = TB(1:3,1:3);
  [n, th] = eje_angulo(RA' * RB);
  nA = RA * n;                                       % eje de giro expresado en {0}
  d = norm(pB - pA);
  tar = P.brazo.tarea;
  wmax = min(P.brazo.qd_max(4:6)); amax_w = min(P.brazo.qdd_max(4:6));
  Tmin = max([1.875 * d / tar.v_cart_max, sqrt(5.7735 * d / tar.a_cart_max), ...
              1.875 * th / wmax, sqrt(5.7735 * th / amax_w), 0.2]);
  T = margen * Tmin;
  info.alargues = 0;
  while true
    N = max(2, ceil(T / dt)); T = N * dt;            % T multiplo del periodo de muestreo (Tema IV §4.6)
    t = (0:N) * dt;
    [s, sd, sdd] = quintico(t, T);
    q = zeros(6, N+1); qd = q; qdd = q;
    qref = q0(:); e_max = 0; dmin = inf;
    for k = 1:N+1
      p = pA + (pB - pA) * s(k);
      R = RA * rot_eje(n, th * s(k));
      qk = mci_brazo([R, p; 0 0 0 1], P, qref);
      if any(isnan(qk)), error('tramo_lin: punto inalcanzable o fuera de limites en s = %.3f', s(k)); end
      Tk = mcd_brazo(qk, P); e_max = max(e_max, norm(Tk(1:3,4) - p));
      [J, inf_j] = jacobiano_brazo(qk, P); dmin = min(dmin, abs(inf_j.det));
      v = [(pB - pA) * sd(k); nA * th * sd(k)];
      a = [(pB - pA) * sdd(k); nA * th * sdd(k)];
      qdk = J \ v;
      hh = 1e-6;
      Jdot = (jacobiano_brazo(qk + hh * qdk, P) - jacobiano_brazo(qk - hh * qdk, P)) / (2 * hh);
      q(:,k) = qk; qd(:,k) = qdk; qdd(:,k) = J \ (a - Jdot * qdk);
      qref = qk;
    end
    ok = all(max(abs(qd), [], 2) <= P.brazo.qd_max + 1e-9) && all(max(abs(qdd), [], 2) <= P.brazo.qdd_max + 1e-9);
    if ok || info.alargues >= 40, break; end
    T = 1.05 * T; info.alargues = info.alargues + 1;
  end
  info.e_max = e_max; info.det_min = dmin; info.d = d; info.theta = th; info.T = T; info.ok = ok;
end

function [n, th] = eje_angulo(R)
  th = acos(max(-1, min(1, (trace(R) - 1) / 2)));
  if th < 1e-9, n = [0; 0; 1]; th = 0; return; end
  n = [R(3,2) - R(2,3); R(1,3) - R(3,1); R(2,1) - R(1,2)] / (2 * sin(th));
end

function R = rot_eje(n, a)
  K = [0 -n(3) n(2); n(3) 0 -n(1); -n(2) n(1) 0];
  R = eye(3) + sin(a) * K + (1 - cos(a)) * K * K;     % Rodrigues (Tema IV §4.9)
end
