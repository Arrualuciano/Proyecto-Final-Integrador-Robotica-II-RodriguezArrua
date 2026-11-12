function A = agv_parametros(P, carga)
  % AGV_PARAMETROS  Modelo del AGV OSOYOO a partir de parametros/parametros.json (bloque agv).
  %   carga: masa adicional centrada [kg] (por defecto 0).
  % Cinematica (patron X, ruedas 1..4 = FL, FR, RL, RR; marco del cuerpo x adelante, y izquierda):
  %   w = H vb,  H = (1/r) [1 -1 -L; 1 1 L; 1 1 -L; 1 -1 L],  L = lx + ly   (Lynch-Park §13.2)
  % Accionamiento (motor CC + reductor, del lado de la rueda; datos de la hoja GM25-370):
  %   tau_i = Kt_i (V_i - Ke w_i) / R - b w_i - tau_c tanh(w_i/eps),  V_i = u_i/255 * Vbat
  % Linealizacion en regimen permanente -> modelo identificable w = K (u - u_dz):
  %   K = (Kt Vbat / (255 R)) / (Kt Ke / R + b),   u_dz = 255 tau_c R / (Kt Vbat)
  if nargin < 1 || isempty(P)
    addpath(fullfile(fileparts(mfilename('fullpath')), '..', '..', 'brazo', 'octave'));
    P = cargar_parametros();
  end
  if nargin < 2, carga = 0; end
  g = P.agv; pl = g.planta; mo = g.motor;
  A.r = g.radio_rueda; A.lx = g.semidistancia_ejes_lx; A.ly = g.semitrocha_ly; A.L = A.lx + A.ly;
  A.H = (1 / A.r) * [1 -1 -A.L; 1 1 A.L; 1 1 -A.L; 1 -1 A.L];
  A.Hp = (A.H' * A.H) \ A.H';                       % pseudoinversa (columnas ortogonales)
  A.carga = carga;
  A.m = g.masa + carga; A.Iz = pl.Iz;              % carga centrada: aporta masa, inercia de giro despreciable
  A.N = mo.reduccion; A.Jw = A.N^2 * pl.J_rotor + pl.J_rueda;
  A.R = mo.R_estimada; A.Ke = mo.Ke_salida_estimada;
  A.Kt = mo.Kt_eta_salida_estimada * pl.dispersion_motores(:);
  A.tc_red = pl.tau_c_reductor; A.tc_rod = pl.tau_c_rodadura; A.b = pl.b;
  A.Vbat = pl.V_bat; A.eps = pl.eps_coulomb;
  A.eta_des = pl.eta_deslizamiento(:);
  A.eta_des(2) = A.eta_des(2) - pl.desliz_lateral_por_kg * carga;
  A.cpr = g.cuentas_por_vuelta_rueda;
  % masa generalizada en coordenadas del cuerpo (restriccion de rodadura w = H vb)
  A.Mb = diag([A.m, A.m, A.Iz]) + A.Jw * (A.H' * A.H);
  A.Mb_inv = inv(A.Mb);
  % controlador (mismos valores que el firmware)
  A.c = g.control;
  Ktn = mo.Kt_eta_salida_estimada;
  A.K_fisico = (Ktn * A.Vbat / (255 * A.R)) / (Ktn * A.Ke / A.R + A.b);
  A.udz_piso = 255 * (A.tc_red + A.tc_rod) * A.R / (Ktn * A.Vbat);
  A.udz_elevado = 255 * A.tc_red * A.R / (Ktn * A.Vbat);
  A.tau_mec_elevado = A.Jw / (Ktn * A.Ke / A.R + A.b);
  % modelo que usa la prealimentacion del controlador (se reemplaza por el identificado)
  A.c.K = A.K_fisico * ones(4,1); A.c.udz = A.udz_piso * ones(4,1);
  A.c.eta_cal = [1; 1; 1];
end
