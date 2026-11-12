% VERIFICAR_POSE_CERO  Comprueba que la tabla DH de parametros.json produce la pose cero
% que usa el CAD: brazo vertical, antebrazo y herramienta horizontales hacia +x0.
% Compara origenes y ejes z de cada trama con los puntos clave O, S, E, W, F, T del CAD.
clear; clc;
aqui = fileparts(mfilename('fullpath'));
addpath(aqui);
P = cargar_parametros();
dh = P.brazo.dh_mat;
d1 = dh(1,2); a2 = dh(2,3); d4 = dh(4,2); d6 = dh(6,2); ht = P.brazo.herramienta.tcp_z6;

[T_tcp, T] = mcd_brazo(zeros(6,1), P);

% Puntos clave esperados (los mismos que calcula el script de Fusion)
S = [0; 0; d1];  E = [0; 0; d1 + a2];  W = [d4; 0; d1 + a2];
F = W + [d6; 0; 0];  Tp = F + [ht; 0; 0];

nombres  = {'O0 (eje 1)', 'O1 (eje 2)', 'O2 (eje 3)', 'O3', 'O4 (muneca)', 'O5 (muneca)', 'O6 (brida)'};
z_esper  = [0 0 1; 0 1 0; 0 1 0; 1 0 0; 0 1 0; 1 0 0; 1 0 0]';   % eje z de cada trama
fprintf('Trama        origen [m]                  eje z\n');
for k = 1:7
  o = T(1:3, 4, k); z = T(1:3, 3, k);
  fprintf('%-12s (%7.4f %7.4f %7.4f)   (%5.2f %5.2f %5.2f)\n', nombres{k}, o, z);
end

err = [ norm(T(1:3,4,2) - S), norm(T(1:3,4,3) - E), norm(T(1:3,4,5) - W), ...
        norm(T(1:3,4,7) - F), norm(T_tcp(1:3,4) - Tp), norm(T(1:3,3,:)(:) - z_esper(:)) ];
fprintf('\nTCP en pose cero: (%.4f, %.4f, %.4f) m\n', T_tcp(1:3,4));
fprintf('Error maximo frente a los puntos clave del CAD: %.2e\n', max(err));
if max(err) < 1e-12
  disp('OK: la tabla DH y la geometria del CAD coinciden en la pose cero.');
else
  disp('ATENCION: la tabla DH NO coincide con la pose cero del CAD.');
end

% Comprobacion extra: los ejes 4, 5 y 6 se cortan en un punto (muneca esferica)
% en una pose generica -> condicion para el desacople cinematico.
q = deg2rad([20; -30; 40; 25; -35; 60]);
[~, T] = mcd_brazo(q, P);
fprintf('Muneca esferica: |O3-O4| = %.2e, |O4-O5| = %.2e (las tramas 4 y 5 deben coincidir con W)\n', ...
        norm(T(1:3,4,5) - T(1:3,4,4) - d4*T(1:3,3,4)), norm(T(1:3,4,5) - T(1:3,4,6)));
