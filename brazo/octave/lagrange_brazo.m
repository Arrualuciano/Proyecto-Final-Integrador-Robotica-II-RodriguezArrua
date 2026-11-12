function [M, c, g] = lagrange_brazo(q, qd, P, dyn, op)
  % LAGRANGE_BRAZO  Modelo por energias (formulacion de Lagrange), independiente de Newton-Euler.
  %   K = 1/2 qd' M qd con  M = sum_k [ m_k Jv_k' Jv_k + Jw_k' I_k Jw_k ] + diag(N^2 Jm)
  %     Jv_k: jacobiano del centro de masa del eslabon k;  Jw_k: jacobiano angular del eslabon k
  %   V = -sum_k m_k g0' c_k   =>  g = dV/dq = -sum_k m_k Jv_k' g0
  %   c_i = sum_j dM_ij/dt qd_j - 1/2 qd' (dM/dq_i) qd     (derivadas de M por diferencias centrales)
  % Sirve para contrastar rne_brazo (Barrientos §5.2 vs §5.3).
  if nargin < 5, op = struct(); end
  M = masa_lag(q, P, dyn, op);
  g = grav_lag(q, P, dyn, op);
  h = 1e-6; dM = zeros(6,6,6);
  for k = 1:6
    e = zeros(6,1); e(k) = h;
    dM(:,:,k) = (masa_lag(q + e, P, dyn, op) - masa_lag(q - e, P, dyn, op)) / (2*h);
  end
  Mdot = zeros(6);
  for k = 1:6, Mdot = Mdot + dM(:,:,k) * qd(k); end
  c = Mdot * qd(:);
  for i = 1:6, c(i) = c(i) - 0.5 * qd(:)' * dM(:,:,i) * qd(:); end
end

function [cs, ms, Is, z, o] = cuerpos(q, P, dyn, op)
  [T_tcp, T] = mcd_brazo(q, P);
  mp = 0; if isfield(op, 'carga'), mp = op.carga; end
  cs = zeros(3,7); ms = zeros(1,7); Is = zeros(3,3,7);
  for i = 1:6
    R = T(1:3,1:3,i+1); L = dyn.link(i);
    ms(i) = L.m; cs(:,i) = T(1:3,4,i+1) + R * L.rc; Is(:,:,i) = R * L.Ic * R';
  end
  % la pieza como un septimo cuerpo, solidario al eslabon 6
  R = T(1:3,1:3,7);
  ms(7) = mp; cs(:,7) = T(1:3,4,7) + R * dyn.carga.rc; Is(:,:,7) = R * dyn.carga.Ic * R' * (mp / dyn.carga.m);
  z = squeeze(T(1:3,3,1:6)); o = squeeze(T(1:3,4,1:6));
end

function M = masa_lag(q, P, dyn, op)
  [cs, ms, Is, z, o] = cuerpos(q, P, dyn, op);
  M = zeros(6);
  for k = 1:7
    j = min(k, 6);                                % la pieza depende de las 6 articulaciones
    Jv = zeros(3,6); Jw = zeros(3,6);
    for i = 1:j
      Jv(:,i) = cross(z(:,i), cs(:,k) - o(:,i)); Jw(:,i) = z(:,i);
    end
    M = M + ms(k) * (Jv' * Jv) + Jw' * Is(:,:,k) * Jw;
  end
  if ~isfield(op, 'rotor') || op.rotor, M = M + diag(dyn.Jref); end
end

function g = grav_lag(q, P, dyn, op)
  gg = 9.81; if isfield(op, 'g'), gg = op.g; end
  g0 = [0; 0; -gg];
  [cs, ms, ~, z, o] = cuerpos(q, P, dyn, op);
  g = zeros(6,1);
  for k = 1:7
    j = min(k, 6);
    for i = 1:j
      g(i) = g(i) - ms(k) * cross(z(:,i), cs(:,k) - o(:,i))' * g0;
    end
  end
end
