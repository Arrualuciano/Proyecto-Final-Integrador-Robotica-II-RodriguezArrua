function [tau, W] = rne_brazo(q, qd, qdd, P, dyn, op)
  % RNE_BRAZO  Dinamica inversa por Newton-Euler recursivo (Tema III, §3.10), vectores en {0}.
  %   tau = rne_brazo(q, qd, qdd, P, dyn, op)   (6x1, N m)
  %   [tau, W] = ...  W(:,i) = [f_i; n_i] (6x6): fuerza y momento que el eslabon i-1 ejerce sobre el
  %   eslabon i en la articulacion i (momento respecto del punto de la articulacion), en {0}.
  %   Sin la inercia del rotor ni la friccion (son internas al accionamiento). Para esfuerzos internos.
  %   op (opcional, campos):
  %     .g        aceleracion gravitatoria [m/s^2]          (9.81; 0 = sin gravedad)
  %     .carga    masa de la pieza en el TCP [kg]           (0)
  %     .rotor    sumar inercia reflejada N^2 Jm qdd        (true)
  %     .friccion sumar b qd + tau_c sgn(qd)                 (false)
  %     .Fext     [f; n] (6x1) que el AMBIENTE ejerce sobre el robot en el TCP, en {0}  (0)
  %   Convencion: M qdd + c + g + tau_f = tau + J^T Fext.
  % Pasada hacia la punta: w_i = w_{i-1} + z qd_i ; al_i = al_{i-1} + z qdd_i + w_{i-1} x (z qd_i)
  %   a_ci = a_Pi + al_i x r_Pc + w_i x (w_i x r_Pc);  a_P(i+1) = a_Pi + al_i x r_PP + w_i x (w_i x r_PP)
  % Pasada hacia la base: F_i = m_i (a_ci - g0);  N_i = I_i al_i + w_i x (I_i w_i)
  %   f_i = F_i + f_{i+1};  n_i = N_i + n_{i+1} + r_Pc x F_i + r_PP x f_{i+1};  tau_i = z^T n_i
  if nargin < 6, op = struct(); end
  g     = campo(op, 'g', 9.81);
  mp    = campo(op, 'carga', 0);
  rotor = campo(op, 'rotor', true);
  fric  = campo(op, 'friccion', false);
  Fext  = campo(op, 'Fext', zeros(6,1));

  [T_tcp, T] = mcd_brazo(q, P);
  g0 = [0; 0; -g];
  z = zeros(3,6); Pp = zeros(3,7); c = zeros(3,6); m = zeros(1,6); I = zeros(3,3,6);
  for i = 1:6
    z(:,i) = T(1:3,3,i); Pp(:,i) = T(1:3,4,i);          % eje y punto de la articulacion i
    L = dyn.link(i);
    if i == 6 && mp > 0                                   % pieza solidaria al eslabon 6
      Lc = dyn.carga; mt = L.m + mp;
      rc = (L.m * L.rc + mp * Lc.rc) / mt;
      d1 = L.rc - rc; d2 = Lc.rc - rc;
      Ic = L.Ic + L.m * ((d1'*d1)*eye(3) - d1*d1') + Lc.Ic * (mp / Lc.m) + mp * ((d2'*d2)*eye(3) - d2*d2');
      L.m = mt; L.rc = rc; L.Ic = Ic;
    end
    R = T(1:3,1:3,i+1);
    m(i) = L.m; c(:,i) = T(1:3,4,i+1) + R * L.rc; I(:,:,i) = R * L.Ic * R';
  end
  Pp(:,7) = T_tcp(1:3,4);

  w = zeros(3,1); al = zeros(3,1); aP = zeros(3,1);
  F = zeros(3,6); N = zeros(3,6);
  for i = 1:6
    w_prev = w;
    w  = w_prev + z(:,i) * qd(i);
    al = al + z(:,i) * qdd(i) + cross(w_prev, z(:,i) * qd(i));
    rPc = c(:,i) - Pp(:,i); rPP = Pp(:,i+1) - Pp(:,i);
    ac = aP + cross(al, rPc) + cross(w, cross(w, rPc));
    aP = aP + cross(al, rPP) + cross(w, cross(w, rPP));
    F(:,i) = m(i) * (ac - g0);
    N(:,i) = I(:,:,i) * al + cross(w, I(:,:,i) * w);
  end

  f = -Fext(1:3); n = -Fext(4:6);                         % condicion de borde en el TCP
  tau = zeros(6,1); W = zeros(6,6);
  for i = 6:-1:1
    rPc = c(:,i) - Pp(:,i); rPP = Pp(:,i+1) - Pp(:,i);
    n = N(:,i) + n + cross(rPc, F(:,i)) + cross(rPP, f);
    f = F(:,i) + f;
    tau(i) = z(:,i)' * n;
    W(:,i) = [f; n];
  end
  if rotor, tau = tau + dyn.Jref .* qdd(:); end
  if fric,  tau = tau + dyn.b .* qd(:) + dyn.tau_c .* tanh(qd(:) / 1e-3); end   % sgn suavizado
end

function v = campo(s, nombre, def)
  if isfield(s, nombre), v = s.(nombre); else, v = def; end
end
