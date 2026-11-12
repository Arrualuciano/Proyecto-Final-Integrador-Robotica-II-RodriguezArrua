function [s, sd, sdd] = quintico(t, T)
  % QUINTICO  Ley temporal de reposo a reposo s(t) en [0,1] (Tema IV, ec. 4.3):
  %   s = 10u^3 - 15u^4 + 6u^5,  u = t/T. Posicion, velocidad y aceleracion nulas en ambos extremos.
  %   Picos: max sd = 15/(8T), max |sdd| = 10*sqrt(3)/(3 T^2).
  u = min(max(t / T, 0), 1);
  s   = u.^3 .* (10 - 15*u + 6*u.^2);
  sd  = (30*u.^2 - 60*u.^3 + 30*u.^4) / T;
  sdd = (60*u - 180*u.^2 + 120*u.^3) / T^2;
  sd(t >= T) = 0; sdd(t >= T) = 0;
end
