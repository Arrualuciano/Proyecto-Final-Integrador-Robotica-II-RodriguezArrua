function [t, q, qd, qdd, T] = tramo_ptp(qA, qB, dt, P, margen)
  % TRAMO_PTP  Tramo articular sincronizado con ley quintica comun (Tema IV §4.2 y §4.6).
  %   Duracion minima que respeta qd_max y qdd_max de cada junta (la junta mas restrictiva fija T):
  %     T >= max_i( 15|dq_i| / (8 V_i),  sqrt(10 sqrt(3) |dq_i| / (3 A_i)) ),  multiplicada por 'margen'
  %   y redondeada hacia ARRIBA a un multiplo de dt (redondear hacia abajo violaria un limite).
  %   Se usa la diferencia CON SIGNO dq = qB - qA (correccion del antecedente PTPq.m, Tema IV §4.13).
  if nargin < 5 || isempty(margen), margen = 1; end
  dq = qB(:) - qA(:);
  V = P.brazo.qd_max; A = P.brazo.qdd_max;
  Tmin = max([15 * abs(dq) ./ (8 * V); sqrt(10 * sqrt(3) * abs(dq) ./ (3 * A)); 0.2]);
  N = max(2, ceil(margen * Tmin / dt)); T = N * dt;
  t = (0:N) * dt;
  [s, sd, sdd] = quintico(t, T);
  q = qA(:) + dq * s; qd = dq * sd; qdd = dq * sdd;
end
