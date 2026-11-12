function [J, info] = jacobiano_brazo(q, P)
  % JACOBIANO_BRAZO  Jacobiano geometrico del TCP en la trama base {0}.
  %   [v; w] = J * qd,  v = velocidad lineal del TCP (m/s), w = velocidad angular (rad/s)
  %   Columna i (articulacion rotacional):  [ z_{i-1} x (p_TCP - o_{i-1}) ;  z_{i-1} ]
  %   (equivale a la propagacion de velocidades base -> herramienta del Tema III)
  %   info.det    : det(J)  (no depende del punto de referencia elegido para v)
  %   info.factores : [rho, cos q3, sin q5]  factores de las tres singularidades
  %   info.det_formula : a2*d4 * rho * cos(q3) * sin(q5) (signo segun la convencion, ver verificacion)
  [T_tcp, T] = mcd_brazo(q, P);
  p = T_tcp(1:3, 4);
  J = zeros(6, 6);
  for i = 1:6
    z = T(1:3, 3, i); o = T(1:3, 4, i);
    J(1:3, i) = cross(z, p - o);
    J(4:6, i) = z;
  end
  M = P.brazo.dh_mat; a2 = M(2,3); d4 = M(4,2);
  rho = a2 * sin(q(2)) + d4 * cos(q(2) + q(3));    % distancia (con signo) de la muneca al eje 1
  info.det = det(J);
  info.factores = [rho, cos(q(3)), sin(q(5))];
  info.det_formula = a2 * d4 * rho * cos(q(3)) * sin(q(5));
end
