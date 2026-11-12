function P = cargar_parametros(ruta)
  % CARGAR_PARAMETROS  Lee parametros/parametros.json (tabla unica del proyecto).
  %   P = cargar_parametros()        busca el archivo relativo a esta carpeta
  %   P = cargar_parametros(ruta)    usa la ruta indicada
  % Agrega campos derivados en SI (rad):
  %   P.brazo.dh_mat  : matriz 6x4 [theta_offset d a alpha] en rad y m
  %   P.brazo.q_min, q_max, qd_max, qdd_max : vectores 6x1 en rad, rad/s, rad/s^2
  if nargin < 1
    aqui = fileparts(mfilename('fullpath'));
    ruta = fullfile(aqui, '..', '..', 'parametros', 'parametros.json');
  end
  P = jsondecode(fileread(ruta));

  dh = P.brazo.dh;
  n = numel(dh);
  M = zeros(n, 4);
  for i = 1:n
    M(i, :) = [deg2rad(dh(i).theta_offset_deg), dh(i).d, dh(i).a, deg2rad(dh(i).alpha_deg)];
  end
  P.brazo.dh_mat = M;

  L = P.brazo.limites;
  P.brazo.q_min   = deg2rad(L.q_min_deg(:));
  P.brazo.q_max   = deg2rad(L.q_max_deg(:));
  P.brazo.qd_max  = deg2rad(L.qd_max_deg_s(:));
  P.brazo.qdd_max = deg2rad(L.qdd_max_deg_s2(:));
end
