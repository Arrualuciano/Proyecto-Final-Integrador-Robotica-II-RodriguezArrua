function A = dh_A(theta, d, a, alpha)
  % DH_A  Transformacion homogenea DH estandar  ^{i-1}A_i = Rz(theta) Tz(d) Tx(a) Rx(alpha).
  % Angulos en rad, longitudes en m. (Las funciones de la catedra en Octave/ usan grados.)
  ct = cos(theta); st = sin(theta);
  ca = cos(alpha); sa = sin(alpha);
  A = [ ct, -st*ca,  st*sa, a*ct;
        st,  ct*ca, -ct*sa, a*st;
         0,     sa,     ca,    d;
         0,      0,      0,    1 ];
end
