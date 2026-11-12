function m = metricas_control(R, C, P, idx, Tref, en_toma)
  % METRICAS_CONTROL  Errores del TCP y esfuerzos de un ensayo de control (enunciado §7.3).
  %   Error de posicion: |p(q) - p(q_ref)|. Error de orientacion: angulo de la rotacion R_ref' R,
  %   theta = acos((tr - 1)/2), que respeta la periodicidad (no compara angulos de Euler).
  %   idx: muestras evaluadas; Tref: poses de referencia en esas muestras; en_toma: mascara de las esperas.
  n = numel(idx); ep = zeros(1,n); ea = zeros(1,n);
  for j = 1:n
    T = mcd_brazo(R.q(:,idx(j)), P);
    ep(j) = norm(T(1:3,4) - Tref(1:3,4,j));
    Re = Tref(1:3,1:3,j)' * T(1:3,1:3);
    ea(j) = acos(max(-1, min(1, (trace(Re) - 1) / 2)));
  end
  m.ep = ep * 1000; m.ea = rad2deg(ea);
  m.e_max = max(m.ep); m.e_rms = sqrt(mean(m.ep.^2)); m.e_fin = m.ep(end);
  m.e_pinza = max(m.ep(en_toma)); m.a_max = max(m.ea); m.a_pinza = max(m.ea(en_toma));
  m.tau_pico = max(abs(R.tau), [], 2)';
  m.sat = 100 * mean(any(R.sat, 1));
  m.e_art_max = max(abs(R.q - C.q), [], 2)';
end
