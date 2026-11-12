function D = preparar_dinamica(P, dyn, mp, escala_masa)
  % PREPARAR_DINAMICA  Datos constantes para dinamica_rapida.m.
  %   mp           masa de la pieza en el TCP [kg] (se suma al eslabon 6)
  %   escala_masa  (opcional) factor sobre masas e inercias de los eslabones: permite construir
  %                un MODELO con error de parametros distinto de la PLANTA (ensayos de incertidumbre).
  if nargin < 3, mp = 0; end
  if nargin < 4, escala_masa = 1; end
  D.dh = P.brazo.dh_mat;
  D.ht = P.brazo.herramienta.tcp_z6;
  D.g0 = [0; 0; -9.81];
  D.m = zeros(1,6); D.rc = zeros(3,6); D.Ic = zeros(3,3,6);
  for i = 1:6
    L = dyn.link(i);
    D.m(i) = escala_masa * L.m; D.rc(:,i) = L.rc; D.Ic(:,:,i) = escala_masa * L.Ic;
  end
  if mp > 0                                   % pieza solidaria al eslabon 6 (Steiner)
    Lc = dyn.carga; m6 = D.m(6); mt = m6 + mp;
    rc = (m6 * D.rc(:,6) + mp * Lc.rc) / mt;
    d1 = D.rc(:,6) - rc; d2 = Lc.rc - rc;
    D.Ic(:,:,6) = D.Ic(:,:,6) + m6 * ((d1'*d1)*eye(3) - d1*d1') + Lc.Ic * (mp / Lc.m) + mp * ((d2'*d2)*eye(3) - d2*d2');
    D.m(6) = mt; D.rc(:,6) = rc;
  end
  D.mp = mp;
  D.Jref = dyn.Jref(:);
  D.b = dyn.b(:); D.tau_c = dyn.tau_c(:);
  D.N = dyn.N(:); D.eta = dyn.eta(:); D.tau_motor_max = dyn.tau_motor_max(:); D.Jm = dyn.Jm(:);
end
