% VERIFICAR_CINEMATICA_AGV  Comprobaciones del modelo cinematico omnidireccional (enunciado §5.2).
clear; clc; more off;
aqui = fileparts(mfilename('fullpath')); addpath(aqui);
dir_res = fullfile(aqui, '..', 'resultados'); if ~exist(dir_res, 'dir'), mkdir(dir_res); end
f_log = fullfile(dir_res, 'verificacion_cinematica_agv.txt'); if exist(f_log, 'file'), delete(f_log); end
diary(f_log);
A = agv_parametros();
fprintf('=== Cinematica mecanum del AGV (%s) ===\n', datestr(now));
fprintf('r = %.4f m, lx = %.4f m, ly = %.4f m, L = lx + ly = %.4f m\n\n', A.r, A.lx, A.ly, A.L);
disp('H [rad/s por (m/s, m/s, rad/s)]:'); disp(A.H);

fprintf('1) Rango de H = %d (3 velocidades del cuerpo, 4 ruedas). H+ H = I con error %.1e\n', rank(A.H), norm(A.Hp * A.H - eye(3)));
n = null(A.H'); n = n / max(abs(n));
fprintf('   Espacio nulo de H^T (combinacion de ruedas sin movimiento del cuerpo): n = %s\n', mat2str(n', 3));
fprintf('   Compatibilidad: una orden de ruedas w es realizable sin deslizar solo si n''w = 0\n');

fprintf('\n2) Casos basicos (v = 0,10 m/s, w = 0,5 rad/s). Ruedas FL, FR, RL, RR [rad/s] y vuelta por H+:\n');
casos = {'avance +x', [0.1; 0; 0]; 'lateral +y (izquierda)', [0; 0.1; 0]; 'diagonal (+x, +y)', [0.1; 0.1; 0];
         'giro antihorario', [0; 0; 0.5]; 'avance + giro', [0.1; 0; 0.5]};
for k = 1:rows(casos)
  w = A.H * casos{k,2}; vb = A.Hp * w;
  fprintf('   %-24s w = [%6.2f %6.2f %6.2f %6.2f]   H+w = (%.3f, %.3f, %.3f)   n''w = %.1e\n', casos{k,1}, w, vb, n' * w);
end
fprintf('   Lectura: en lateral +y, FL y RR giran hacia atras y FR y RL hacia adelante; en diagonal dos ruedas quedan quietas.\n');

fprintf('\n3) Orden de ruedas incompatible: w = [5 5 5 0] rad/s\n');
w = [5; 5; 5; 0]; vb = A.Hp * w; res = w - A.H * vb;
fprintf('   H+ w = (%.3f m/s, %.3f m/s, %.3f rad/s); residuo w - H H+ w = %s rad/s (las ruedas "pelean": deslizan los rodillos)\n', vb, mat2str(res', 3));

wmax = A.K_fisico * (255 - A.udz_piso);
fprintf('\n4) Limite de rueda wmax = K (255 - u_dz) = %.2f rad/s (con Vbat = %.1f V). Conjunto admisible del cuerpo:\n', wmax, A.Vbat);
fprintf('   |vx| + |vy| + L |w| <= r wmax = %.3f m/s  (octaedro)\n', A.r * wmax);
fprintf('   Maximos: avance %.3f m/s, lateral %.3f m/s, diagonal |v| = %.3f m/s, giro %.2f rad/s\n', ...
        A.r * wmax, A.r * wmax, sqrt(2) * A.r * wmax / 2, A.r * wmax / A.L);
vb = [0.30; 0.20; 1.0]; w = A.H * vb; s = max(abs(w)) / wmax;
fprintf('   Orden (0,30; 0,20; 1,0) -> max|w| = %.2f rad/s: escala comun %.3f -> (%.3f, %.3f, %.3f), conserva la direccion\n', ...
        max(abs(w)), 1 / s, vb / s);

fprintf('\n5) Odometria: con incrementos de rueda dphi (encoders), ds = r dphi y [dx dy dth] = H+ (dphi) ...\n');
fprintf('   integrada con el rumbo en el punto medio; los factores de calibracion eta escalan dx, dy, dth.\n');
diary off;
