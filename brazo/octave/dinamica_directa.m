function qdd = dinamica_directa(q, qd, tau, P, dyn, op)
  % DINAMICA_DIRECTA  qdd = M(q) \ (tau + J^T Fext - c - g - tau_f)   (Tema III, ec. 3.2)
  % Se resuelve el sistema lineal sin formar M^-1. op: campos de rne_brazo.
  if nargin < 6, op = struct(); end
  % RNE con qdd = 0 da c + g + tau_f - J^T Fext  (todo lo que no depende de qdd)
  h = rne_brazo(q, qd, zeros(6,1), P, dyn, op);
  op0 = op; op0.g = 0; op0.friccion = false; op0.Fext = zeros(6,1);
  M = zeros(6);
  for j = 1:6
    e = zeros(6,1); e(j) = 1;
    M(:, j) = rne_brazo(q, zeros(6,1), e, P, dyn, op0);
  end
  qdd = M \ (tau(:) - h);
end
