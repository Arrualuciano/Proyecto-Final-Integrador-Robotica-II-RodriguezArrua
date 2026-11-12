function [q, Q, info] = mci_brazo(T, P, q_ref)
  % MCI_BRAZO  Cinematica inversa analitica del brazo 6R por desacople cinematico.
  %   T     : ^0T_TCP deseada (4x4), m
  %   P     : parametros (cargar_parametros)
  %   q_ref : 6x1 configuracion de referencia (la actual) para elegir la solucion mas
  %           cercana y desenrollar angulos. Por defecto zeros(6,1).
  %   q     : 6x1 solucion elegida (NaN si ninguna es alcanzable dentro de limites)
  %   Q     : 6x8 todas las soluciones (columnas con NaN si no son alcanzables)
  %   info  : .config (3x8: hombro, codo, muneca = +1/-1), .valida (1x8, dentro de limites),
  %           .elegida (indice), .singular (texto si hubo una singularidad resuelta por convencion)
  %
  % Desarrollo (convencion DH de parametros.json, a1 = a3 = 0):
  %   1) Centro de muneca: pw = p - (d6 + h) z,  z = 3ra columna de R.
  %   2) q1 = atan2(pw_y, pw_x)  (hombro adelante, rho = +r)   o  q1 + pi (hombro atras, rho = -r)
  %      con r = sqrt(pw_x^2 + pw_y^2) y s = pw_z - d1.
  %   3) En el plano del brazo: rho = a2 sin q2 + d4 cos(q2+q3),  s = a2 cos q2 - d4 sin(q2+q3)
  %      => sin q3 = (a2^2 + d4^2 - rho^2 - s^2) / (2 a2 d4)   (dos ramas: signo de cos q3)
  %      => q2 = atan2(k1 rho - k2 s, k2 rho + k1 s),  k1 = a2 - d4 sin q3,  k2 = d4 cos q3
  %   4) ^3R_6 = (^0R_3)^T R, con
  %      ^3R_6(:,3) = [-c4 s5; -s4 s5; c5],  ^3R_6(3,1:2) = [s5 c6, -s5 s6]
  %      => q5 = atan2(+-sqrt(r13^2 + r23^2), r33),  q4 = atan2(-r23/s5, -r13/s5),  q6 = atan2(-r32/s5, r31/s5)
  %   Singularidades: r = 0 (hombro), cos q3 = 0 (codo), sin q5 = 0 (muneca).
  if nargin < 3 || isempty(q_ref), q_ref = zeros(6,1); end
  q_ref = q_ref(:);
  M  = P.brazo.dh_mat;
  d1 = M(1,2); a2 = M(2,3); d4 = M(4,2); d6 = M(6,2);
  h  = P.brazo.herramienta.tcp_z6;
  tol = 1e-9;

  R = T(1:3,1:3); p = T(1:3,4);
  pw = p - (d6 + h) * R(:,3);
  r = hypot(pw(1), pw(2)); s = pw(3) - d1;

  Q = nan(6, 8); config = zeros(3, 8); singular = '';
  k = 0;
  for hombro = [1, -1]
    if r < tol
      q1 = q_ref(1); singular = 'hombro (muneca sobre el eje 1): q1 = q1 de referencia';
    else
      q1 = atan2(pw(2), pw(1)) + (hombro < 0) * pi;
    end
    rho = hombro * r;
    D = (a2^2 + d4^2 - rho^2 - s^2) / (2 * a2 * d4);
    for codo = [1, -1]
      for muneca = [1, -1]
        k = k + 1; config(:, k) = [hombro; codo; muneca];
        if abs(D) > 1 + 1e-12, continue; end          % fuera de alcance
        D_ = max(-1, min(1, D));
        q3 = atan2(D_, codo * sqrt(1 - D_^2));
        k1 = a2 - d4 * sin(q3); k2 = d4 * cos(q3);
        q2 = atan2(k1 * rho - k2 * s, k2 * rho + k1 * s);   % ya es la variable articular (incluye el offset DH)
        % orientacion
        T03 = eye(4);
        qq = [q1; q2; q3];
        for i = 1:3
          T03 = T03 * dh_A(qq(i) + M(i,1), M(i,2), M(i,3), M(i,4));
        end
        R36 = T03(1:3,1:3)' * R;
        s5 = muneca * sqrt(R36(1,3)^2 + R36(2,3)^2);
        q5 = atan2(s5, R36(3,3));
        if abs(s5) < 1e-7
          q4 = q_ref(4);
          if R36(3,3) > 0      % c5 = +1: R36(1:2,1:2) depende de q4 + q6
            q6 = atan2(R36(2,1), R36(1,1)) - q4;
          else                 % c5 = -1: depende de q4 - q6
            q6 = q4 - atan2(-R36(2,1), -R36(1,1));
          end
          singular = 'muneca (sin q5 = 0): q4 = q4 de referencia';
        else
          q4 = atan2(-R36(2,3) / s5, -R36(1,3) / s5);
          q6 = atan2(-R36(3,2) / s5, R36(3,1) / s5);
        end
        Q(:, k) = [q1; q2; q3; q4; q5; q6];
      end
    end
  end

  % Desenrollar respecto de q_ref (continuidad) y verificar limites
  valida = false(1, 8);
  for k = 1:8
    if any(isnan(Q(:,k))), continue; end
    qk = q_ref + atan2(sin(Q(:,k) - q_ref), cos(Q(:,k) - q_ref));   % equivalente mas cercano
    for i = 1:6                                                       % si sale de limites, probar +-2pi
      if qk(i) > P.brazo.q_max(i), qk(i) -= 2*pi; end
      if qk(i) < P.brazo.q_min(i), qk(i) += 2*pi; end
    end
    Q(:, k) = qk;
    valida(k) = all(qk >= P.brazo.q_min - 1e-9) && all(qk <= P.brazo.q_max + 1e-9);
  end

  q = nan(6, 1); elegida = 0;
  if any(valida)
    dist = inf(1, 8);
    W = [1; 1; 1; 0.5; 0.5; 0.5];          % pondera mas los ejes grandes (mueven mas masa)
    for k = find(valida)
      dist(k) = sum(W .* (Q(:,k) - q_ref).^2);
    end
    [~, elegida] = min(dist);
    q = Q(:, elegida);
  end
  info = struct('config', config, 'valida', valida, 'elegida', elegida, 'singular', singular, ...
                'pw', pw, 'r', r, 's', s);
end
