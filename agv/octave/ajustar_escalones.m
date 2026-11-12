function R = ajustar_escalones(t, u, w, usar)
  % AJUSTAR_ESCALONES  Identificacion del accionamiento de cada rueda a partir de escalones de PWM
  % (enunciado §5.3): modelo equivalente PWM -> velocidad
  %     w_ss = K (|u| - u_dz) sgn(u)      (regimen permanente, |u| > u_dz)
  %     G(s) = K / (tau s + 1)            (transitorio: 63 % del cambio)
  %   t: 1xN [s]; u: 4xN PWM aplicado; w: 4xN velocidad medida [rad/s]; usar: indices de escalones a usar
  %   (para ajustar con unos y verificar con otros). Devuelve por rueda: K, udz, tau, tabla de escalones.
  % Metodo: se detectan los tramos de u constante distinto de cero; w_ss = media del ultimo 40 % del tramo;
  % regresion lineal por minimos cuadrados de w_ss contra |u| (positivos y negativos juntos, con signo);
  % tau = tiempo hasta 63 % de w_ss desde el inicio del tramo.
  if nargin < 4, usar = []; end
  for i = 1:4
    ui = u(i,:); wi = w(i,:);
    cambio = [true, diff(ui) ~= 0];
    ini = find(cambio); fin = [ini(2:end) - 1, numel(ui)];
    E = [];                                   % filas: u, w_ss, tau, t0
    for k = 1:numel(ini)
      a = ini(k); b = fin(k);
      if ui(a) == 0 || (t(b) - t(a)) < 0.5, continue; end
      cola = a + round(0.6 * (b - a)) : b;
      wss = mean(wi(cola));
      w0 = wi(max(a - 1, 1));
      obj = w0 + 0.632 * (wss - w0);
      j = find(sign(wss - w0) * (wi(a:b) - obj) >= 0, 1);
      tau = NaN; if ~isempty(j), tau = t(a + j - 1) - t(a); end
      E(end+1, :) = [ui(a), wss, tau, t(a)];
    end
    R(i).escalones = E;
    if isempty(E), R(i).K = NaN; R(i).udz = NaN; R(i).tau = NaN; continue; end
    sel = 1:rows(E); if ~isempty(usar), sel = intersect(usar, sel); end
    Es = E(sel, :);
    mov = abs(Es(:,2)) > 0.2;                  % escalones que vencieron la zona muerta
    x = abs(Es(mov,1)); y = abs(Es(mov,2));
    p = [x, ones(size(x))] \ y;               % y = K x - K udz
    R(i).K = p(1); R(i).udz = -p(2) / p(1);
    R(i).tau = median(Es(mov & ~isnan(Es(:,3)), 3));
    R(i).w_pred = @(uu) sign(uu) .* max(R(i).K * (abs(uu) - R(i).udz), 0);
  end
end
