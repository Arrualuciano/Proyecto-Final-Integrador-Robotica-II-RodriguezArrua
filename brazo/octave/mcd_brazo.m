function [T_tcp, T] = mcd_brazo(q, P)
  % MCD_BRAZO  Modelo cinematico directo del brazo 6R.
  %   q : 6x1 [rad]   P : estructura de cargar_parametros()
  %   T_tcp : ^0T_TCP (4x4), herramienta incluida (traslacion tcp_z6 sobre z6)
  %   T     : 4x4x7, T(:,:,k) = ^0T_{k-1}, k = 1..7  (T(:,:,1) = identidad = trama 0)
  M = P.brazo.dh_mat;
  T = zeros(4, 4, 7);
  T(:, :, 1) = eye(4);
  for i = 1:6
    T(:, :, i+1) = T(:, :, i) * dh_A(q(i) + M(i,1), M(i,2), M(i,3), M(i,4));
  end
  T_herr = eye(4); T_herr(3, 4) = P.brazo.herramienta.tcp_z6;
  T_tcp = T(:, :, 7) * T_herr;
end
