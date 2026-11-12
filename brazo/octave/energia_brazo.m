function [E, K, V] = energia_brazo(q, qd, P, dyn, op)
  % ENERGIA_BRAZO  Energia mecanica K + V del brazo (incluye la pieza si op.carga > 0 y los rotores).
  % V se mide con altura cero en el plano z0 = 0.
  if nargin < 5, op = struct(); end
  M = matrices_dinamicas(q, zeros(6,1), P, dyn, op);
  K = 0.5 * qd(:)' * M * qd(:);
  gg = 9.81; if isfield(op, 'g'), gg = op.g; end
  mp = 0; if isfield(op, 'carga'), mp = op.carga; end
  [T_tcp, T] = mcd_brazo(q, P);
  V = 0;
  for i = 1:6
    c = T(1:3,4,i+1) + T(1:3,1:3,i+1) * dyn.link(i).rc;
    V = V + dyn.link(i).m * gg * c(3);
  end
  cp = T(1:3,4,7) + T(1:3,1:3,7) * dyn.carga.rc;
  V = V + mp * gg * cp(3);
  E = K + V;
end
