function [M, c, g] = matrices_dinamicas(q, qd, P, dyn, op)
  % MATRICES_DINAMICAS  M(q), c(q,qd) y g(q) evaluando Newton-Euler (metodo de Walker-Orin):
  %   g        = RNE(q, 0, 0)                 con gravedad
  %   c        = RNE(q, qd, 0)                sin gravedad
  %   M(:,j)   = RNE(q, 0, e_j)               sin gravedad (incluye N^2 Jm si op.rotor)
  % op: mismos campos que rne_brazo (se ignora op.friccion y op.Fext aqui).
  if nargin < 5, op = struct(); end
  op.friccion = false; op.Fext = zeros(6,1);
  g = rne_brazo(q, zeros(6,1), zeros(6,1), P, dyn, op);
  op0 = op; op0.g = 0;
  c = rne_brazo(q, qd, zeros(6,1), P, dyn, op0);
  M = zeros(6);
  for j = 1:6
    e = zeros(6,1); e(j) = 1;
    M(:, j) = rne_brazo(q, zeros(6,1), e, P, dyn, op0);
  end
end
