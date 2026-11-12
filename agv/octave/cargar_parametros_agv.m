function P = cargar_parametros_agv()
  % CARGAR_PARAMETROS_AGV  Misma tabla unica que el brazo (parametros/parametros.json).
  addpath(fullfile(fileparts(mfilename('fullpath')), '..', '..', 'brazo', 'octave'));
  P = cargar_parametros();
end
