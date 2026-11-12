function R = identificar_lab(carpeta)
  % IDENTIFICAR_LAB  Ajusta K, u_dz y tau por rueda con un registro del laboratorio.
  %   carpeta: sesion de agv/pc/agv_consola.py o de la interfaz (contiene telemetria.csv), grabada con
  %   el guion agv/pc/guiones/identificacion_escalones.txt (robot elevado).
  % Ajusta con los escalones impares y verifica con los pares (datos distintos, enunciado §5.3).
  % Imprime las ordenes MDL para cargar el modelo en el firmware.
  M = csvread(fullfile(carpeta, 'telemetria.csv'), 1, 0);    % salta el encabezado
  % columnas: t_pc, t_ms, modo, falla, x, y, phi, xr, yr, phir, vx, vy, wz, wr0..3, wm0..3, u0..3, c0..3, ...
  t = M(:,2)' / 1000;
  w = M(:,18:21)' / 1000;            % wm [rad/s]
  u = M(:,22:25)';
  todos = ajustar_escalones(t, u, w);
  n = rows(todos(1).escalones);
  R = ajustar_escalones(t, u, w, 1:2:n);
  fprintf('Rueda   K [rad/s/PWM]   u_dz [PWM]   tau [s]   error de verificacion (escalones pares) [rad/s]\n');
  nombres = {'FL', 'FR', 'RL', 'RR'};
  for i = 1:4
    E = todos(i).escalones; v = 2:2:rows(E);
    err = sqrt(mean((R(i).w_pred(E(v,1)) - E(v,2)).^2));
    fprintf('  %s      %.4f        %6.1f      %.3f      %.3f\n', nombres{i}, R(i).K, R(i).udz, R(i).tau, err);
  end
  fprintf('\nOrdenes para el firmware (ruedas elevadas: en el piso la zona muerta crece por la rodadura):\n');
  for i = 1:4, fprintf('MDL %d %.4f %.1f\n', i - 1, R(i).K, R(i).udz); end
end
