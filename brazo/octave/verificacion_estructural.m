% VERIFICACION_ESTRUCTURAL  Resistencia y deformacion del eslabon L2 (brazo superior), pieza impresa critica.
%   Enunciado §4.2: verificar resistencia y deformacion de al menos un elemento estructural critico, con
%   propiedades del material impreso que contemplen la direccion de las capas.
%   Metodo analitico (viga de pared delgada) con las cargas internas exactas de Newton-Euler:
%     1. Cargas internas en L2 con rne_brazo (fuerza y momento en las articulaciones 2 y 3).
%     2. Casos: C1 ciclo de trabajo con la carga maxima (dinamico), C2 peor postura estatica (sostenida,
%        fluencia lenta), C3 colision en el TCP con el motor 2 saturado (accidental).
%     3. Seccion: solo perimetros y capas solidas (el relleno no se cuenta para resistencia).
%     4. Orientaciones: 'acostado' (capas normales al eje de la articulacion) vs 'parado' (capas normales
%        al eje del eslabon); cada tension contra la resistencia de su direccion.
%     5. Union atornillada de las dos mitades (L2 no entra entero en una cama de 220 x 220 mm).
%     6. Deformacion: desplazamiento del TCP por flexion y torsion de L2.
%   Salida: brazo/resultados/verificacion_estructural.txt y .mat
more off;

% ================================================================ funciones
function escribir(fid, varargin)
  fprintf(1, varargin{:}); fprintf(fid, varargin{:});
end

function r = ifelse(c, a, b)
  if c, r = a; else, r = b; end
end

function sec = tubo(by, bz, ty, tz)
  % tubo rectangular de pared delgada by x bz; pared normal a y2 de espesor ty, normal a z2 de espesor tz
  sec.by = by; sec.bz = bz; sec.ty = ty; sec.tz = tz;
  sec.A  = by*bz - (by-2*ty)*(bz-2*tz);
  sec.Iy = (by*bz^3 - (by-2*ty)*(bz-2*tz)^3)/12;      % respecto de y2 (la tension varia con z2)
  sec.Iz = (bz*by^3 - (bz-2*tz)*(by-2*ty)^3)/12;      % respecto de z2 (la tension varia con y2)
  sec.Am = (by-ty)*(bz-tz);                            % area media (Bredt)
  sec.J  = 4*sec.Am^2 / (2*(by-ty)/tz + 2*(bz-tz)/ty);
end

function [Fl, Ml] = seccion_local(q, W, s, P, lam, a2, g0)
  % Esfuerzos que la parte inferior de L2 ejerce sobre la superior en la seccion s, en el marco {2}
  % (filas x2, y2, z2). Columna 1: desde la raiz (desprecia el cubo del hombro, cerca de S).
  % Columna 2: desde el codo (desprecia el motor M3 y la inercia del tramo; masa repartida en el largo).
  [~, T] = mcd_brazo(q, P);
  R2 = T(1:3,1:3,3); S = T(1:3,4,2); E = T(1:3,4,3);
  p = S + s * R2(:,1);
  Fa = W(1:3,2); Ma = W(4:6,2) + cross(S - p, W(1:3,2));
  m = lam * (a2 - s); c = p + 0.5 * (a2 - s) * R2(:,1);
  Fb = W(1:3,3) - m * g0; Mb = W(4:6,3) + cross(E - p, W(1:3,3)) - cross(c - p, m * g0);
  Fl = [R2' * Fa, R2' * Fb]; Ml = [R2' * Ma, R2' * Mb];
end

function [FS, sig, tau] = evaluar(Fl, Ml, Nc, o)
  % tension normal en la esquina mas cargada, corte en cada tipo de pared, FS cuadratico (Tsai-Hill)
  sec = o.sec; FS = Inf; sig = 0; tau = 0;
  for k = 1:columns(Fl)
    N = abs(Fl(1,k)) + Nc; Vy = abs(Fl(2,k)); Vz = abs(Fl(3,k));
    T = abs(Ml(1,k)); My = abs(Ml(2,k)); Mz = abs(Ml(3,k));
    s = N/sec.A + My/(sec.Iy/(sec.bz/2)) + Mz/(sec.Iz/(sec.by/2));
    tp = T/(2*sec.Am*sec.ty) + 1.5*Vz/(2*sec.bz*sec.ty);   % paredes normales a y2
    tc = T/(2*sec.Am*sec.tz) + 1.5*Vy/(2*sec.by*sec.tz);   % paredes normales a z2
    f = 1 / sqrt((s/o.s_x)^2 + max(tp/o.t_perim, tc/o.t_capa)^2);
    if f < FS, FS = f; sig = s; tau = max(tp, tc); end
  end
end

function [FS, info] = fs_secciones(q, W, Nc, P, lam, a2, g0, geo, orient)
  % FS en: (1) raiz acostado, (2) raiz parado, (3) extremos del solape (media seccion, acostado)
  FS = Inf(3,1); info = struct('s', 0, 'sig', 0, 'tau', 0);
  [Fl, Ml] = seccion_local(q, W, geo.s_raiz, P, lam, a2, g0);
  [FS(1), info.sig, info.tau] = evaluar(Fl, Ml, Nc, orient(1));
  FS(2) = evaluar(Fl, Ml, Nc, orient(2));
  for s = geo.s_ens
    [Fl, Ml] = seccion_local(q, W, s, P, lam, a2, g0);
    FS(3) = min(FS(3), evaluar(Fl, Ml, Nc, orient(3)));
  end
end

function F = fuerzas_bulones(Fl, Ml, geo)
  % [corte; traccion] por tornillo. En el plano del solape (x2-y2): N, Vy y Mz (cupla entre los 2
  % tornillos separados p). Fuera del plano: Vz, My (palanca p) y torsion (palanca medio ancho).
  N = max(abs(Fl(1,:))); Vy = max(abs(Fl(2,:))); Vz = max(abs(Fl(3,:)));
  T = max(abs(Ml(1,:))); My = max(abs(Ml(2,:))); Mz = max(abs(Ml(3,:)));
  F = [hypot(N/2, Vy/2 + Mz/geo.p_bul); Vz/2 + My/geo.p_bul + T/geo.by];
end

function dtcp = deformacion(q, W, P, lam, a2, g0, geo, imp, mat, con_relleno)
  % integra la curvatura (flexion en y2 y z2) y el giro por torsion a lo largo de L2 y los traslada al TCP
  [Ttcp, T] = mcd_brazo(q, P); R2 = T(1:3,1:3,3); S = T(1:3,4,2); ptcp = Ttcp(1:3,4);
  ds = 1e-3; dtcp = zeros(3,1);
  for s = geo.s_raiz + ds/2 : ds : a2
    if s < 0.12,      bz = geo.bz; dz = 0;     nb = 1;   % viga 40 x 44
    elseif s < 0.16,  bz = 0.090;  dz = 0;     nb = 1;   % base de la horquilla del codo
    else,             bz = 0.020;  dz = 0.035; nb = 2;   % orejas de la horquilla
    end
    sc = tubo(geo.by, bz, imp.t_perim, imp.t_capa);
    Iy = sc.Iy; Iz = sc.Iz;
    if con_relleno                                     % nucleo de relleno con modulo reducido
      bi = geo.by - 2*imp.t_perim; hi = bz - 2*imp.t_capa;
      Iy = Iy + mat.E_relleno * bi*hi^3/12; Iz = Iz + mat.E_relleno * hi*bi^3/12;
    end
    Iy = nb*(Iy + sc.A*dz^2); Iz = nb*Iz; J = nb*sc.J;
    [~, Ml] = seccion_local(q, W, s, P, lam, a2, g0);
    M = Ml(:,1);
    dth = R2 * [M(1)/(mat.G*J); M(2)/(mat.E*Iy); M(3)/(mat.E*Iz)] * ds;
    dtcp = dtcp + cross(dth, ptcp - (S + s * R2(:,1)));
  end
end

% ================================================================ datos
aqui = fileparts(mfilename('fullpath')); addpath(aqui);
res = fullfile(aqui, '..', 'resultados');
P = cargar_parametros(); dyn = modelo_dinamico_brazo(P);
g0 = [0; 0; -9.81];

% PLA por FDM (parametros.json, brazo.material). Eryildiz (2021), Eur. Mech. Sci. 5(3):116-120: traccion
% 55,5 MPa (plano, filamentos a 0 grados) y 35,5 MPa (parado: carga perpendicular a las capas). Valores de
% diseno = 80 % (dispersion de filamentos e impresoras, envejecimiento). Chacon et al. (2017), Mater. Des.
% 124:143-157: misma tendencia.
R = P.brazo.material.resistencia; I = P.brazo.material.impresion;
mat.nombre = [P.brazo.material.nombre ' (FDM)'];
mat.s_xy = R.traccion_plano;       % traccion en el plano de la capa, a lo largo de los filamentos [Pa]
mat.s_z  = R.traccion_entre_capas; % traccion perpendicular a las capas [Pa]
mat.t_xy = mat.s_xy / sqrt(3);     % corte en el plano de la capa (von Mises) [Pa]
mat.t_z  = mat.s_z / sqrt(3);      % corte sobre la interfaz entre capas [Pa]
mat.E    = R.E_plano;              % modulo en el plano [Pa]
mat.G    = mat.E / (2 * (1 + R.nu));
mat.E_relleno = R.E_relleno_relativo;   % modulo relativo del relleno (estimado)
imp.t_perim = I.perimetros * I.ancho_linea;
imp.t_capa  = I.capas_solidas * I.altura_capa;
imp.relleno = I.relleno;
FS_req = [3 4 2];                  % C1 dinamico / C2 sostenido (fluencia lenta) / C3 accidental

% Geometria de L2 (parametros.json, cad). Marco {2}: origen en el codo E, x2 de S a E, z2 = eje de las
% articulaciones 2 y 3. s = distancia desde el hombro S a lo largo de x2.
a2 = 0.20;
geo.by = 0.040; geo.bz = 0.044;    % viga 40 mm (y2) x 44 mm (z2)
geo.s_raiz = 0.035;                % fin del cubo del hombro (radio 35 mm)
% union de las dos mitades (parametros.json, eslabon L2_brazo, 'division')
esl = P.brazo.cad.eslabones; if ~iscell(esl), esl = num2cell(esl); end
div = esl{cellfun(@(e) strcmp(e.nombre, 'L2_brazo'), esl)}.division;
ag = div.agujeros; if ~iscell(ag), ag = num2cell(ag); end
s_bul = sort(cellfun(@(h) h.centro(3), ag(cellfun(@(h) strncmp(h.uso, 'tornillo', 8), ag))));
geo.s_ens  = [div.s_inicio div.s_fin];   % extremos del solape
geo.s_j    = mean(s_bul);                % centro de la union
geo.p_bul  = diff(s_bul);                % separacion entre los 2 tornillos M4
geo.d_bul  = 0.004;
geo.e_bul  = min(s_bul(1) - div.s_inicio, div.s_fin - s_bul(2));   % distancia al extremo del solape
r_polea = 0.030;                   % radio supuesto de la polea de salida del eje 3 (correa desde M3)

orient(1).nombre = 'acostado';     % capas normales a z2: perimetros en las caras y2 = +-20 mm
orient(1).sec = tubo(geo.by, geo.bz, imp.t_perim, imp.t_capa);
orient(1).s_x = mat.s_xy;          % sigma_x en el plano de la capa
orient(1).t_perim = mat.t_z;       % corte en paredes de perimetro: actua sobre las interfaces
orient(1).t_capa  = mat.t_xy;      % corte en las capas solidas: en el plano
orient(2).nombre = 'parado';       % capas normales a x2: todas las caras son perimetros
orient(2).sec = tubo(geo.by, geo.bz, imp.t_perim, imp.t_perim);
orient(2).s_x = mat.s_z;           % sigma_x atraviesa las capas
orient(2).t_perim = mat.t_z; orient(2).t_capa = mat.t_z;
orient(3) = orient(1); orient(3).nombre = 'media seccion del solape';
orient(3).sec = tubo(geo.by, geo.bz/2, imp.t_perim, imp.t_capa);

m_L2 = dyn.link(2).m_estructura; lam = m_L2 / (a2 + geo.s_raiz);
mpmax = P.brazo.carga_max;
cic = load(fullfile(res, 'ciclo_brazo.mat')); C = cic.C; n = columns(C.q);

% ================================================================ C1: ciclo con la carga maxima
FS1 = Inf(3, n); M1 = zeros(6, n); sig1 = zeros(1, n);
for k = 1:n
  mp = mpmax * (C.mp(k) > 0);
  [tau, W] = rne_brazo(C.q(:,k), C.qd(:,k), C.qdd(:,k), P, dyn, struct('carga', mp, 'rotor', false));
  [FS1(:,k), inf1] = fs_secciones(C.q(:,k), W, 2*abs(tau(3))/r_polea, P, lam, a2, g0, geo, orient);
  sig1(k) = inf1.sig;
  [Fl, Ml] = seccion_local(C.q(:,k), W, geo.s_raiz, P, lam, a2, g0);
  M1(:,k) = [Fl(:,1); Ml(:,1)];
end
[~, k1] = min(FS1(1,:));

% ================================================================ C2: peor postura estatica
FS2 = Inf(3,1); q_C2 = zeros(6,1);
for a = deg2rad(-90:5:110)
  for b = deg2rad(-110:5:140)
    q = [0; a; b; 0; 0; 0];
    [tau, W] = rne_brazo(q, zeros(6,1), zeros(6,1), P, dyn, struct('carga', mpmax, 'rotor', false));
    f = fs_secciones(q, W, 2*abs(tau(3))/r_polea, P, lam, a2, g0, geo, orient);
    if f(1) < FS2(1), q_C2 = q; end
    FS2 = min(FS2, f);
  end
end

% ================================================================ C3: colision en el TCP, motor 2 saturado
% El motor 2 entrega su par maximo sin perdidas (conservador) contra un obstaculo en el TCP: fuerza
% perpendicular a (TCP - S), en el plano del brazo, tal que z1 . ((TCP - S) x F) = tau_stall.
tau_stall = dyn.N(2) * dyn.tau_motor_max(2);
k_col = find(C.seg == 6, 1, 'last'); k_tom = find(C.seg == 2, 1, 'last');
poses_C3 = {q_C2, zeros(6,1), C.q(:,k_col)};
FS3 = Inf(3,1); Fc3 = 0; W3 = [];
for i = 1:numel(poses_C3)
  q = poses_C3{i};
  [Ttcp, T] = mcd_brazo(q, P); z1 = T(1:3,3,2); r = Ttcp(1:3,4) - T(1:3,4,2);
  d = cross(z1, r); d = d / norm(d); Fmag = tau_stall / norm(r);
  for sg = [-1 1]
    for mp = [0 mpmax]
      [~, W] = rne_brazo(q, zeros(6,1), zeros(6,1), P, dyn, ...
                         struct('carga', mp, 'rotor', false, 'Fext', [sg*Fmag*d; 0; 0; 0]));
      f = fs_secciones(q, W, 0, P, lam, a2, g0, geo, orient);
      if f(1) < FS3(1), Fc3 = Fmag; W3 = W; q_C3 = q; end
      FS3 = min(FS3, f);
    end
  end
end

% ================================================================ union atornillada (centro del solape)
% Solape a media altura (z2): cada mitad conserva 22 mm; 2 tornillos M4 8.8 pasantes en z2 (= direccion
% de construccion: agujeros verticales), tuercas y arandelas DIN 9021.
s_j = geo.s_j; Fj = zeros(2,3);
for k = 1:n
  mp = mpmax * (C.mp(k) > 0);
  [~, W] = rne_brazo(C.q(:,k), C.qd(:,k), C.qdd(:,k), P, dyn, struct('carga', mp, 'rotor', false));
  [Fl, Ml] = seccion_local(C.q(:,k), W, s_j, P, lam, a2, g0);
  Fj(:,1) = max(Fj(:,1), fuerzas_bulones(Fl, Ml, geo));
end
[~, W] = rne_brazo(q_C2, zeros(6,1), zeros(6,1), P, dyn, struct('carga', mpmax, 'rotor', false));
[Fl, Ml] = seccion_local(q_C2, W, s_j, P, lam, a2, g0); Fj(:,2) = fuerzas_bulones(Fl, Ml, geo);
[Fl, Ml] = seccion_local(q_C3, W3, s_j, P, lam, a2, g0); Fj(:,3) = fuerzas_bulones(Fl, Ml, geo);
A_s = 8.78e-6; f_ub = 800e6;                          % M4 clase 8.8
Fv_Rd = 0.6*f_ub*A_s/1.25; Ft_Rd = 0.9*f_ub*A_s/1.25;  % EN 1993-1-8
t_mitad = geo.bz/2;
aplast   = Fj(1,:) / (geo.d_bul * t_mitad);           % aplastamiento del agujero (en el plano de la capa)
desgarro = Fj(1,:) / (2 * geo.e_bul * t_mitad);       % desgarro hacia el borde (corte en el plano)
F_pre = 1000; mu = 0.30; F_fric = mu * F_pre;         % precarga ~0,8 N m; capacidad por tornillo
holgura = 0.2e-3;                                     % juego diametral tipico de un agujero impreso

% ================================================================ deformacion
casos_def = {'toma (sin pieza)', C.q(:,k_tom), 0; 'colocacion (pieza 0,10 kg)', C.q(:,k_col), P.pieza.masa; ...
             'colocacion (carga max 0,25 kg)', C.q(:,k_col), mpmax; 'peor postura C2 (0,25 kg)', q_C2, mpmax};
d_sin = zeros(3, rows(casos_def)); d_con = d_sin;
for i = 1:rows(casos_def)
  [~, W] = rne_brazo(casos_def{i,2}, zeros(6,1), zeros(6,1), P, dyn, struct('carga', casos_def{i,3}, 'rotor', false));
  d_sin(:,i) = deformacion(casos_def{i,2}, W, P, lam, a2, g0, geo, imp, mat, false);
  d_con(:,i) = deformacion(casos_def{i,2}, W, P, lam, a2, g0, geo, imp, mat, true);
end
% diagrama de momentos a lo largo de L2 en la peor postura (para la figura)
s_diag = linspace(geo.s_raiz, a2, 60); M_diag = zeros(3, 60);
[~, W] = rne_brazo(q_C2, zeros(6,1), zeros(6,1), P, dyn, struct('carga', mpmax, 'rotor', false));
for i = 1:60, [~, Ml] = seccion_local(q_C2, W, s_diag(i), P, lam, a2, g0); M_diag(:,i) = Ml(:,1); end

% ================================================================ informe
fid = fopen(fullfile(res, 'verificacion_estructural.txt'), 'w');
escribir(fid, '=== Verificacion estructural del eslabon L2 (brazo superior) ===\n');
escribir(fid, 'Material: %s | s_xy = %.1f MPa, s_z = %.1f MPa, t_xy = %.1f MPa, t_z = %.1f MPa, E = %.1f GPa\n', ...
         mat.nombre, mat.s_xy/1e6, mat.s_z/1e6, mat.t_xy/1e6, mat.t_z/1e6, mat.E/1e9);
escribir(fid, 'Impresion: 4 perimetros (%.1f mm), capas solidas %.1f mm, relleno %d %% (no se cuenta para resistencia)\n', ...
         imp.t_perim*1e3, imp.t_capa*1e3, round(100*imp.relleno));
for o = 1:3
  sc = orient(o).sec;
  escribir(fid, '  seccion %-26s %2.0f x %2.0f mm: A = %4.0f mm2, Iy = %6.0f mm4, Iz = %6.0f mm4, J = %6.0f mm4\n', ...
           orient(o).nombre, sc.by*1e3, sc.bz*1e3, sc.A*1e6, sc.Iy*1e12, sc.Iz*1e12, sc.J*1e12);
end
escribir(fid, 'Masa de la estructura de L2: %.3f kg | par de bloqueo del eje 2: N2 tau_m,max = %.1f N m\n', m_L2, tau_stall);
escribir(fid, '\nFS minimo [raiz acostado | raiz parado | solape (acostado)]   requerido\n');
escribir(fid, 'C1 ciclo, carga %.2f kg (dinamica completa)   %5.1f | %5.1f | %5.1f        %d\n', mpmax, min(FS1, [], 2), FS_req(1));
escribir(fid, 'C2 peor postura estatica, %.2f kg            %5.1f | %5.1f | %5.1f        %d  (q2, q3) = (%.0f, %.0f) deg\n', ...
         mpmax, FS2, FS_req(2), rad2deg(q_C2(2)), rad2deg(q_C2(3)));
escribir(fid, 'C3 colision en el TCP, motor 2 saturado     %5.1f | %5.1f | %5.1f        %d  F_contacto = %.1f N\n', ...
         FS3, FS_req(3), Fc3);
escribir(fid, 'C1 peor instante t = %.2f s (tramo %d): N = %.1f N, Vy = %.1f N, Vz = %.1f N, T = %.2f, My = %.2f, Mz = %.2f N m; sigma = %.2f MPa\n', ...
         C.t(k1), C.seg(k1), M1(:,k1), sig1(k1)/1e6);
escribir(fid, '\nUnion atornillada (2 x M4 8.8 en s = %.0f y %.0f mm, solape %.0f..%.0f mm, %.0f mm por mitad, borde %.0f mm):\n', ...
         1e3*(geo.s_j - geo.p_bul/2), 1e3*(geo.s_j + geo.p_bul/2), 1e3*geo.s_ens, t_mitad*1e3, 1e3*geo.e_bul);
nj = {'C1 ciclo', 'C2 sostenida', 'C3 colision'};
for i = 1:3
  escribir(fid, '  %-12s corte %6.1f N (FS bulon %5.1f) | traccion %6.1f N (FS %5.1f) | aplastamiento %5.2f MPa (FS %5.1f) | desgarro %5.2f MPa (FS %5.1f)\n', ...
           nj{i}, Fj(1,i), Fv_Rd/Fj(1,i), Fj(2,i), Ft_Rd/Fj(2,i), aplast(i)/1e6, mat.s_xy/aplast(i), ...
           desgarro(i)/1e6, mat.t_xy/desgarro(i));
end
escribir(fid, '  Friccion con precarga %.0f N por tornillo (mu = %.2f): %.0f N por tornillo -> en el ciclo %s\n', ...
         F_pre, mu, F_fric, ifelse(Fj(1,1) < F_fric, 'no desliza', 'DESLIZA'));
escribir(fid, '  Si desliza con juego %.1f mm: giro %.1f mrad -> hasta %.1f mm en el TCP (brazo de palanca 0,33 m)\n', ...
         holgura*1e3, 1e3*holgura/geo.p_bul, 1e3*holgura/geo.p_bul*0.33);
escribir(fid, '\nDesplazamiento del TCP por la deformacion de L2 (solo paredes | con relleno):\n');
for i = 1:rows(casos_def)
  escribir(fid, '  %-32s |d| = %5.2f mm | %5.2f mm   d = [%6.2f %6.2f %6.2f] mm\n', casos_def{i,1}, ...
           1e3*norm(d_sin(:,i)), 1e3*norm(d_con(:,i)), 1e3*d_sin(:,i));
end
escribir(fid, '  Colocacion con pieza 0,10 kg: %.2f mm de error no medido por los encoders (R-B-4: 1 mm)\n', 1e3*norm(d_sin(:,2)));
fclose(fid);
t_ciclo = C.t;
save('-mat7-binary', fullfile(res, 'verificacion_estructural.mat'), 'FS1', 'M1', 'sig1', 'mat', 'imp', 'geo', ...
     'q_C2', 'FS2', 'FS3', 'Fj', 'd_sin', 'd_con', 'tau_stall', 's_diag', 'M_diag', 'FS_req', 't_ciclo');
