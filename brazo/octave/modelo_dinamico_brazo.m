function dyn = modelo_dinamico_brazo(P, mostrar)
  % MODELO_DINAMICO_BRAZO  Parametros inerciales de los eslabones 1..6 del brazo.
  %  - Estructura impresa: masa, centro e inercia desde los STL exportados por Fusion
  %    (brazo/stl/<eslabon>__estructura*.stl, mm) con la densidad EFECTIVA del JSON.
  %  - Motores: prismas macizos con la masa del JSON (envolventes del CAD).
  %  - Todo se calcula en la trama {0} en la pose cero y se lleva a la trama DH del eslabon:
  %      r_c^i = R_i^T (c - o_i),   I_c^i = R_i^T I_c R_i   (constantes)
  %  - dyn.carga: pieza en el TCP (cubo); se suma al eslabon 6 cuando se pide.
  % Salida dyn.link(i): .m [kg], .rc [m, trama i], .Ic [kg m^2, central, ejes de la trama i], .nombre
  if nargin < 2, mostrar = false; end
  aqui = fileparts(mfilename('fullpath'));
  dir_stl = fullfile(aqui, '..', 'stl');
  rho = P.brazo.material.densidad_efectiva;
  [T_tcp0, T0] = mcd_brazo(zeros(6,1), P);
  K.O = [0;0;0]; K.S = T0(1:3,4,2); K.E = T0(1:3,4,3); K.W = T0(1:3,4,5);
  K.F = T0(1:3,4,7); K.T = T_tcp0(1:3,4);

  esl = P.brazo.cad.eslabones;
  if ~iscell(esl), esl = num2cell(esl); end
  for i = 1:6
    e = esl{i + 1};                                    % esl{1} es la base fija
    % todas las piezas impresas del eslabon: '<eslabon>__estructura.stl' o, si esta dividido
    % (L2), '<eslabon>__estructura_a.stl' + '_b.stl'. Las integrales de volumen se suman.
    arch = dir(fullfile(dir_stl, [e.nombre '__estructura*.stl']));
    if isempty(arch), error('Falta el STL de la estructura de %s en %s', e.nombre, dir_stl); end
    tri = [];
    for k = 1:numel(arch)
      tri = [tri; stl_leer(fullfile(dir_stl, arch(k).name)) / 1000];         % mm -> m
    end
    [ms, cs, Is, Vs] = propiedades_masa_malla(tri, rho);
    partes = struct('nombre', 'estructura', 'm', ms, 'c', cs, 'I', Is);
    prim = e.primitivas; if ~iscell(prim), prim = num2cell(prim); end
    for k = 1:numel(prim)
      pr = prim{k};
      if ~strcmp(pr.tipo, 'motor'), continue; end
      c = K.(pr.ref) + pr.centro(:); t = pr.tamano(:); m = pr.masa;
      I = m / 12 * diag([t(2)^2 + t(3)^2, t(1)^2 + t(3)^2, t(1)^2 + t(2)^2]);
      partes(end+1) = struct('nombre', ['motor_' pr.id], 'm', m, 'c', c, 'I', I);
    end
    [m, c, I] = combinar_masas(partes);
    R = T0(1:3,1:3,i+1); o = T0(1:3,4,i+1);
    dyn.link(i) = struct('nombre', e.nombre, 'm', m, 'rc', R' * (c - o), 'Ic', R' * I * R, ...
                         'c0', c, 'm_estructura', ms, 'V_estructura', Vs, 'partes', partes);
  end

  % Pieza (cubo) en el TCP, expresada en la trama 6
  a = P.pieza.lado; mp = P.pieza.masa;
  dyn.carga = struct('m', mp, 'rc', [0; 0; P.brazo.herramienta.tcp_z6], 'Ic', mp * a^2 / 6 * eye(3), 'lado', a);

  ac = P.brazo.accionamientos;
  dyn.N = ac.N(:); dyn.eta = ac.eta(:); dyn.Jm = ac.Jm(:);
  dyn.tau_motor_max = ac.tau_motor_max(:);
  dyn.Jref = dyn.N.^2 .* dyn.Jm;                       % inercia del rotor reflejada a la junta
  dyn.b = P.brazo.friccion.b(:); dyn.tau_c = P.brazo.friccion.tau_c(:);

  if mostrar
    fprintf('Eslabon            masa[kg]  (estructura)   r_c en trama i [mm]           diag(I_c) [kg cm^2]\n');
    for i = 1:6
      L = dyn.link(i);
      fprintf('%-17s %8.4f  (%6.4f)   (%7.1f %7.1f %7.1f)   (%7.3f %7.3f %7.3f)\n', L.nombre, L.m, ...
              L.m_estructura, 1000 * L.rc, 1e4 * diag(L.Ic));
    end
    fprintf('Masa movil total (eslabones 1-6): %.3f kg; pieza %.3f kg\n', sum([dyn.link.m]), mp);
  end
end

function [m, c, I] = combinar_masas(partes)
  % Suma de cuerpos rigidos: masa total, centro comun y tensor central por Steiner.
  m = sum([partes.m]);
  c = zeros(3,1);
  for k = 1:numel(partes), c = c + partes(k).m * partes(k).c; end
  c = c / m;
  I = zeros(3);
  for k = 1:numel(partes)
    d = partes(k).c - c;
    I = I + partes(k).I + partes(k).m * ((d' * d) * eye(3) - d * d');
  end
end
