function [m, c, Ic, V] = propiedades_masa_malla(tri, rho)
  % PROPIEDADES_MASA_MALLA  Masa, centro de masa y tensor de inercia central de un solido
  % cerrado dado por su malla triangular (n x 9, en m), con densidad uniforme rho (kg/m^3).
  % Metodo: se descompone el solido en tetraedros (origen, a, b, c) con volumen con signo
  %   V_k = a.(b x c)/6,  centro (a+b+c)/4,
  %   S_k = int x x^T dV = V_k/20 [a a' + b b' + c c' + (a+b+c)(a+b+c)'].
  % Luego I_O = tr(S) I3 - S, y se traslada al centro: I_c = I_O - m[(c'c) I3 - c c'] (Steiner).
  % Es un calculo independiente del de Fusion (sirve para contrastar).
  a = tri(:, 1:3); b = tri(:, 4:6); cc = tri(:, 7:9);
  v6 = dot(a, cross(b, cc, 2), 2);            % 6 V_k
  V = sum(v6) / 6;
  if V < 0, v6 = -v6; V = -V; end              % normales hacia adentro: invertir
  s = a + b + cc;
  c = (sum(v6 .* s, 1) / (24 * V))';
  w = v6 / 120;
  S = (a .* w)' * a + (b .* w)' * b + (cc .* w)' * cc + (s .* w)' * s;
  m = rho * V;
  IO = rho * (trace(S) * eye(3) - S);
  Ic = IO - m * ((c' * c) * eye(3) - c * c');
  Ic = (Ic + Ic') / 2;
end
