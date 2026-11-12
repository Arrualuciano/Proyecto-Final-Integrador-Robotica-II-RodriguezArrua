function tri = stl_leer(ruta)
  % STL_LEER  Lee un STL binario. Devuelve tri (n x 9): [v1 v2 v3] por triangulo,
  % en las unidades del archivo (los STL de Fusion de este proyecto estan en mm).
  fid = fopen(ruta, 'r');
  if fid < 0, error('No se pudo abrir %s', ruta); end
  fread(fid, 80, 'uint8');
  n = fread(fid, 1, 'uint32');
  d = fread(fid, [12, n], '12*float32=>double', 2);   % normal (3) + 3 vertices (9), salta 2 bytes
  fclose(fid);
  tri = d(4:12, :)';
end
